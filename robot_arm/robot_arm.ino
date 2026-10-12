// 5-DOF robot arm + claw on a PCA9685, with synchronized trapezoidal moves,
// inverse kinematics, and serial-terminal commands (type "?" for help).
// Units: degrees, cm, seconds.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

using namespace BLA;

// ============================================================
// Configuration
// ============================================================

const int SERVO_FREQ = 50;                  // Hz
const unsigned long UPDATE_PERIOD_US = 20000;  // one servo frame
const int NUM_JOINTS = 5;

struct JointConfig {
    int   channel;
    float minAngle, maxAngle;   // joint limits
    float offset;               // servo angle at joint angle 0
    bool  reversed;
    int   minPulse, maxPulse;   // PCA9685 counts at 0 and SERVO_RANGE deg
    float maxVel, maxAcc;       // deg/s, deg/s^2
};

// Degrees the servo turns between minPulse and maxPulse.
const float SERVO_RANGE = 225;

// Offsets are calibrated from the home pose (upper arm vertical, forearm and
// gripper pointing forward = joint angles 0, 90, -90, 90, 0), which sits at
// servo angles 128.75, 125, 212.5, 137.5, 118.75 (on the 0-225 scale):
//   offset = servo - joint angle   (reversed: servo + joint angle)
// min/max are the joints' physical ranges; where a servo cannot cover its
// whole range, jointMin()/jointMax() narrow it (with a warning at startup).
const JointConfig JOINTS[NUM_JOINTS] = {
//   ch    min    max    offset  rev   minP maxP  vel acc
    { 0,  -90,    90,  128.75,  false, 120, 500,  60, 120 },  // J1 base
    { 1,    0,   220,   35,     false, 120, 500,  45,  80 },  // J2 shoulder
    { 2, -120,    90,  122.5,   true,  120, 500,  60, 120 },  // J3 elbow
    { 3, -100,   120,  227.5,   true,  120, 500,  90, 180 },  // J4 wrist pitch
    { 4,  -90,    85,  118.75,  false, 120, 500,  90, 180 },  // J5 wrist roll
};

// Usable joint limits: the configured limits, narrowed to what the servo's
// 0-SERVO_RANGE travel can actually reach (servo = offset +/- joint angle).
// Everything (moves, IK, FK) uses these, so the stored pose always matches
// what was sent to the servos.
float jointMin(int i) {
    const JointConfig& c = JOINTS[i];
    return max(c.minAngle, c.reversed ? c.offset - SERVO_RANGE : 0 - c.offset);
}
float jointMax(int i) {
    const JointConfig& c = JOINTS[i];
    return min(c.maxAngle, c.reversed ? c.offset : SERVO_RANGE - c.offset);
}

// Claw: channel, open angle, closed angle, min pulse, max pulse
const int   CLAW_CHANNEL = 5;
const float CLAW_OPEN = 25, CLAW_CLOSED = 112.5;
const int   CLAW_MIN_PULSE = 120, CLAW_MAX_PULSE = 500;

// DH table: r, alpha (rad), d
const float DH[NUM_JOINTS][3] = {
    {  0.0f, PI / 2, 11.5f },
    { 10.5f, 0,       0.0f },
    { 10.0f, 0,       0.0f },
    {  0.0f, PI / 2,  0.0f },
    {  0.0f, 0,      15.0f },
};

// Home: servo angles 103, 100, 170, 110, 95 on channels 0-4.
const float HOME[NUM_JOINTS] = { 0, 90, -90, 90, 0 };

Adafruit_PWMServoDriver pwm;

// ============================================================
// Servo output
// ============================================================

// Servo angle (0..SERVO_RANGE) -> pulse count.
// Float math instead of map(), which rounds to whole degrees.
void writeServo(int channel, float servoAngle, int minPulse, int maxPulse) {
    servoAngle = constrain(servoAngle, 0.0f, SERVO_RANGE);
    pwm.setPWM(channel, 0, minPulse + servoAngle / SERVO_RANGE * (maxPulse - minPulse) + 0.5f);
}

// Wrap an angle into [lo, lo + 360) deg, so joints whose range goes past
// +-180 (e.g. a shoulder that turns 0..220) get the right representation.
float wrapFrom(float a, float lo) {
    while (a >= lo + 360) a -= 360;
    while (a <  lo)       a += 360;
    return a;
}

// ============================================================
// Trapezoidal profile
//
// Normalized: s(t) goes 0 -> 1. Each joint follows
//   q_i(t) = start_i + s(t) * (target_i - start_i)
// so all joints start and stop together. Short moves become
// triangular (no cruise phase).
// ============================================================

class TrapezoidalProfile {
public:
    // vmax, amax: normalized limits (fraction of move per s, per s^2)
    void plan(float vmax, float amax) {
        A = amax;
        if (vmax * vmax / amax >= 1) {          // triangular
            ta = sqrtf(1 / amax);
            Vp = amax * ta;
            T  = 2 * ta;
        } else {                                // trapezoidal
            Vp = vmax;
            ta = vmax / amax;
            T  = 1 / vmax + vmax / amax;
        }
    }

    float duration() const { return T; }

    float position(float t) const {
        if (t <= 0) return 0;
        if (t >= T) return 1;
        if (t < ta)     return 0.5f * A * t * t;               // accelerate
        if (t < T - ta) return 0.5f * A * ta * ta + Vp * (t - ta);  // cruise
        float r = T - t;
        return 1 - 0.5f * A * r * r;                           // decelerate
    }

private:
    float A = 0, Vp = 0, ta = 0, T = 0;
};

// ============================================================
// RobotArm
// ============================================================

class RobotArm {
public:
    // Direct jump, no profile. Use at startup: with no servo feedback
    // the initial pose is unknown, so there is nothing to plan from.
    void setJointAngles(const float q[NUM_JOINTS]) {
        moving = false;
        apply(q);
    }

    // Start a synchronized trapezoidal move (non-blocking; call update()).
    void moveTo(const float target[NUM_JOINTS]) {
        // The most constrained joint sets the pace:
        // joint i moving D deg needs ds/dt <= maxVel/D and d2s/dt2 <= maxAcc/D.
        float v = INFINITY, a = INFINITY;

        for (int i = 0; i < NUM_JOINTS; i++) {
            start[i]  = angle[i];
            target_[i] = constrain(target[i], jointMin(i), jointMax(i));
            float D = fabs(target_[i] - start[i]);
            if (D > 0.05f) {
                v = min(v, JOINTS[i].maxVel / D);
                a = min(a, JOINTS[i].maxAcc / D);
            }
        }

        if (isinf(v)) { moving = false; return; }   // already there

        profile.plan(v, a);
        startUs = micros();
        lastUs  = startUs - UPDATE_PERIOD_US;   // first step immediately
        moving  = true;
    }

    // Halt immediately; the arm holds its last commanded pose.
    void stop() { moving = false; }

    // Call every loop(); steps the move once per UPDATE_PERIOD_US.
    void update() {
        unsigned long now = micros();
        if (!moving || now - lastUs < UPDATE_PERIOD_US) return;
        lastUs = now;

        float t = (now - startUs) * 1e-6f;
        if (t >= profile.duration()) moving = false;

        float s = profile.position(t);
        float q[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++)
            q[i] = start[i] + s * (target_[i] - start[i]);
        apply(q);
    }

    bool isMoving() const { return moving; }

    // Analytic inverse kinematics for one solution branch.
    //   x, y, z : tool position in the base frame [cm]
    //   pitch   : theta2 + theta3 + theta4 [deg]. With the DH table above the
    //             gripper points (pitch - 90) deg from horizontal:
    //             pitch 0 = straight down, pitch 90 = straight out.
    //   roll    : theta5 [deg]
    //   branch  : bit 0 set = base turns away from the target and the arm
    //             reaches back over the top; bit 1 = elbow sign.
    // Up to 4 branches exist; joint limits rule some out. Writes joint angles
    // to q and returns false if this branch is out of reach or out of limits.
    // Link lengths come from the DH table.
    bool solveBranch(float x, float y, float z, float pitch, float roll,
                     int branch, float q[NUM_JOINTS]) const {
        const float d1 = DH[0][2], a2 = DH[1][0], a3 = DH[2][0],
                    a4 = DH[3][0], d5 = DH[4][2];

        // Base: face the target (u = +radius), or face away (u = -radius).
        float q1 = atan2(y, x) * RAD_TO_DEG;
        float u  = sqrt(x * x + y * y);
        if (branch & 1) { q1 += (q1 > 0) ? -180 : 180; u = -u; }

        // Joints 2-4 now form a planar chain (u = radial, v = height above
        // joint 2). Remove the fixed link-4 + tool offset to get the joint-4
        // position: offset = a4 * x4 + d5 * z4, with x4 = (c, s), z4 = (s, -c).
        float phi = pitch * DEG_TO_RAD;
        float uw = u - (a4 * cos(phi) + d5 * sin(phi));
        float vw = (z - d1) - (a4 * sin(phi) - d5 * cos(phi));

        // Two-link solution (law of cosines) for joints 2 and 3.
        float c3 = (uw * uw + vw * vw - a2 * a2 - a3 * a3) / (2 * a2 * a3);
        if (fabs(c3) > 1.001f) return false;    // out of reach
        c3 = constrain(c3, -1.0f, 1.0f);        // absorb rounding at full stretch

        float t3 = ((branch & 2) ? 1 : -1) * acos(c3);
        float t2 = atan2(vw, uw) - atan2(a3 * sin(t3), a2 + a3 * cos(t3));

        q[0] = q1;
        q[1] = wrapFrom(t2 * RAD_TO_DEG, jointMin(1));
        q[2] = wrapFrom(t3 * RAD_TO_DEG, jointMin(2));
        q[3] = wrapFrom(pitch - q[1] - q[2], jointMin(3));
        q[4] = roll;

        for (int i = 0; i < NUM_JOINTS; i++)
            if (q[i] < jointMin(i) - 0.01f || q[i] > jointMax(i) + 0.01f) return false;
        return true;
    }

    // IK using whichever valid branch is closest to the current pose.
    bool inverseKinematics(float x, float y, float z, float pitch, float roll,
                           float q[NUM_JOINTS]) const {
        float cand[NUM_JOINTS];
        float bestCost = INFINITY;
        for (int b = 0; b < 4; b++) {
            if (!solveBranch(x, y, z, pitch, roll, b, cand)) continue;
            float cost = 0;
            for (int i = 0; i < NUM_JOINTS; i++) cost += fabs(cand[i] - angle[i]);
            if (cost < bestCost) {
                bestCost = cost;
                for (int i = 0; i < NUM_JOINTS; i++) q[i] = cand[i];
            }
        }
        return !isinf(bestCost);
    }

    // Solve IK and start a trapezoidal move. False if unreachable (no motion).
    bool moveToPose(float x, float y, float z, float pitch, float roll) {
        float q[NUM_JOINTS];
        if (!inverseKinematics(x, y, z, pitch, roll, q)) return false;
        moveTo(q);
        return true;
    }

    void openClaw()  { writeServo(CLAW_CHANNEL, CLAW_OPEN,   CLAW_MIN_PULSE, CLAW_MAX_PULSE); }
    void closeClaw() { writeServo(CLAW_CHANNEL, CLAW_CLOSED, CLAW_MIN_PULSE, CLAW_MAX_PULSE); }

    // T05 = A1 * A2 * A3 * A4 * A5 (standard DH)
    Matrix<4, 4> forwardKinematics() const {
        Matrix<4, 4> T = {          // identity
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        for (int i = 0; i < NUM_JOINTS; i++) {
            float th = angle[i] * DEG_TO_RAD;
            float r = DH[i][0], al = DH[i][1], d = DH[i][2];
            float ct = cos(th), st = sin(th), ca = cos(al), sa = sin(al);
            Matrix<4, 4> A = {
                ct, -st * ca,  st * sa, r * ct,
                st,  ct * ca, -ct * sa, r * st,
                0,   sa,       ca,      d,
                0,   0,        0,       1
            };
            T = T * A;
        }
        return T;
    }

    void printState() const {
        Matrix<4, 4> T = forwardKinematics();
        Serial.print("q = ");
        for (int i = 0; i < NUM_JOINTS; i++) { Serial.print(angle[i]); Serial.print(' '); }
        Serial.print(" | XYZ = ");
        Serial.print(T(0, 3)); Serial.print(", ");
        Serial.print(T(1, 3)); Serial.print(", ");
        Serial.print(T(2, 3));
        Serial.print(" cm | pitch = ");
        Serial.println(angle[1] + angle[2] + angle[3]);
    }

private:
    float angle[NUM_JOINTS] = {};      // last commanded joint angles
    float start[NUM_JOINTS] = {};
    float target_[NUM_JOINTS] = {};
    TrapezoidalProfile profile;
    bool moving = false;
    unsigned long startUs = 0, lastUs = 0;

    // Clamp, convert joint -> servo angle, and send.
    void apply(const float q[NUM_JOINTS]) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            const JointConfig& c = JOINTS[i];
            angle[i] = constrain(q[i], jointMin(i), jointMax(i));
            float servo = c.reversed ? c.offset - angle[i] : c.offset + angle[i];
            writeServo(c.channel, servo, c.minPulse, c.maxPulse);
        }
    }
};

// ============================================================
// Block pick and place (assignment)
//
// Coordinates typed by the user are in the robot's base frame [cm], as on
// the course graph paper: origin at the base's rotation axis, x_0 to the
// right along the paper (the direction the base faces at J1 = 0), y_0 up
// the page. If you measure from somewhere else, set BASE_X/BASE_Y to where
// the base axis sits in your coordinates.
// A block's z is the height of its bottom face: a block on the table is
// z = 0, the second block in a stack is z = BLOCK_SIZE, and so on.
// ============================================================

RobotArm robot;

const float BASE_X = 0, BASE_Y = 0;     // base axis in typed coordinates [cm]
const float BLOCK_SIZE = 2.5;           // block edge length [cm]
const float GRIP_Z = BLOCK_SIZE / 2;    // tool point height above block bottom
const float CLEARANCE = 4;              // approach height above the grip [cm]
const float PLACE_X = 5, PLACE_Y = 5, PLACE_Z = 0;  // drop-off (paper coords)
const int   MAX_STACK = 3;              // tower height at the drop-off
const float MAX_TILT = 60;              // gripper may tilt this far from vertical
const float TILT_STEP = 5;              // pitch search step [deg]
const float TILT_SPACING = 15;          // min pitch gap between listed solutions
const int   MAX_SOLUTIONS = 6;

// One IK solution for a grasp: the joint angles at the block and directly
// above it, both with the same branch and gripper pitch.
struct GraspSolution {
    int   branch;
    float pitch;
    float qGrip[NUM_JOINTS];
    float qAbove[NUM_JOINTS];
};

// Solutions for a block at paper position (px, py, pz). The arm has one
// more joint than a position needs, so besides the branch (base direction,
// elbow up/down) the gripper pitch is free. For each branch, pitches from
// -MAX_TILT to +MAX_TILT are tried; one that reaches both the grip point and
// the point CLEARANCE above it is a solution, and further solutions in the
// same branch are kept if they differ by at least TILT_SPACING.
int findGraspSolutions(float px, float py, float pz, GraspSolution sol[MAX_SOLUTIONS]) {
    float x = px - BASE_X, y = py - BASE_Y;     // paper -> robot frame
    float z = pz + GRIP_Z;
    int n = 0;
    for (int b = 0; b < 4; b++) {
        float last = -1000;
        for (float pitch = -MAX_TILT; pitch <= MAX_TILT && n < MAX_SOLUTIONS; pitch += TILT_STEP) {
            if (pitch - last < TILT_SPACING) continue;
            GraspSolution& s = sol[n];
            if (robot.solveBranch(x, y, z, pitch, 0, b, s.qGrip) &&
                robot.solveBranch(x, y, z + CLEARANCE, pitch, 0, b, s.qAbove)) {
                s.branch = b;
                s.pitch  = pitch;
                last = pitch;
                n++;
            }
        }
    }
    return n;
}

// The single best solution (gripper closest to vertical), for the drop-off.
bool findBestGrasp(float px, float py, float pz, GraspSolution& best) {
    float x = px - BASE_X, y = py - BASE_Y, z = pz + GRIP_Z;
    for (int k = 0; k <= 2 * (int)(MAX_TILT / TILT_STEP); k++) {
        float pitch = ((k + 1) / 2) * TILT_STEP * ((k % 2) ? 1 : -1);  // 0, 5, -5, ...
        for (int b = 0; b < 4; b++) {
            if (robot.solveBranch(x, y, z, pitch, 0, b, best.qGrip) &&
                robot.solveBranch(x, y, z + CLEARANCE, pitch, 0, b, best.qAbove)) {
                best.branch = b;
                best.pitch  = pitch;
                return true;
            }
        }
    }
    return false;
}

void printSolution(int k, const GraspSolution& s) {
    // Elbow "up" = elbow above the line from shoulder to wrist.
    bool back = s.branch & 1;
    bool elbowUp = (s.qGrip[2] < 0) != back;
    Serial.print(F("  ")); Serial.print(k + 1);
    Serial.print(back ? F(": base reaches back, ") : F(": base faces block, "));
    Serial.print(elbowUp ? F("elbow up, pitch ") : F("elbow down, pitch "));
    Serial.print(s.pitch, 0);
    Serial.print(F(" deg | q ="));
    for (int i = 0; i < NUM_JOINTS; i++) { Serial.print(' '); Serial.print(s.qGrip[i], 1); }
    Serial.println();
}

// Motion plan: list of joint-space moves, each with a claw action on arrival.
enum ClawAction { NONE, OPEN, CLOSE };

struct Step {
    float q[NUM_JOINTS];
    ClawAction claw;            // done on arrival
    unsigned long dwellMs;      // pause after arrival
};

const int MAX_STEPS = 8;
Step steps[MAX_STEPS];
int numSteps = 0;
int step = 0;
bool runSequence = false;       // true while a pick and place runs
bool arrived = false;
unsigned long arrivedMs = 0;

int stackCount = 0;             // blocks already on the tower

GraspSolution pending[MAX_SOLUTIONS];  // solutions for the last block typed
int numPending = 0;

void addStep(const float q[NUM_JOINTS], ClawAction claw, unsigned long dwellMs) {
    Step& s = steps[numSteps++];
    for (int i = 0; i < NUM_JOINTS; i++) s.q[i] = q[i];
    s.claw = claw;
    s.dwellMs = dwellMs;
}

// "[x, y, z]": list the IK solutions for a block there.
void listSolutions(float x, float y, float z) {
    numPending = findGraspSolutions(x, y, z, pending);
    Serial.print(numPending);
    Serial.print(F(" solution(s) for block at ["));
    Serial.print(x); Serial.print(F(", ")); Serial.print(y); Serial.print(F(", "));
    Serial.print(z); Serial.println(F("]:"));
    for (int k = 0; k < numPending; k++) printSolution(k, pending[k]);
    if (numPending > 0) {
        Serial.print(F("Type 1"));
        if (numPending > 1) { Serial.print(F("-")); Serial.print(numPending); }
        Serial.println(F(" to pick it up with that solution."));
    }
}

// "k": pick up the listed block with solution k and stack it at the drop-off.
void runSolution(int k) {
    if (k < 1 || k > numPending) {
        Serial.println(F("no such solution - type the block's [x, y, z] first"));
        return;
    }
    if (stackCount >= MAX_STACK) {
        Serial.println(F("tower is full - clear it and type r"));
        return;
    }

    // Drop-off: the next free spot on the tower.
    GraspSolution place;
    float placeZ = PLACE_Z + stackCount * BLOCK_SIZE;
    if (!findBestGrasp(PLACE_X, PLACE_Y, placeZ, place)) {
        Serial.println(F("drop-off position unreachable - nothing moved"));
        return;
    }

    const GraspSolution& pick = pending[k - 1];
    numSteps = 0;
    addStep(pick.qAbove,   OPEN,  300);     // above block, open claw
    addStep(pick.qGrip,    CLOSE, 800);     // down, grip
    addStep(pick.qAbove,   NONE,  200);     // lift
    addStep(HOME,          NONE,    0);     // travel high
    addStep(place.qAbove,  NONE,  200);     // above tower
    addStep(place.qGrip,   OPEN,  800);     // down, release
    addStep(place.qAbove,  NONE,  200);     // back up
    addStep(HOME,          NONE,    0);

    Serial.print(F("Picking with solution ")); Serial.println(k);
    step = 0;
    runSequence = true;
    arrived = false;
    robot.moveTo(steps[0].q);
}

// move -> arrive (claw action) -> dwell -> next step
void sequenceStep() {
    if (robot.isMoving()) return;

    if (!arrived) {
        arrived = true;
        arrivedMs = millis();
        if (steps[step].claw == OPEN)  robot.openClaw();
        if (steps[step].claw == CLOSE) robot.closeClaw();
        return;
    }

    if (millis() - arrivedMs < steps[step].dwellMs) return;

    if (++step >= numSteps) {
        runSequence = false;
        stackCount++;
        numPending = 0;                     // block has moved; re-enter next one
        Serial.print(F("Done. Tower height: "));
        Serial.println(stackCount);
        return;
    }
    arrived = false;
    robot.moveTo(steps[step].q);
}

// ============================================================
// Serial commands (115200 baud, newline-terminated)
//
//   j q1 q2 q3 q4 q5        move to joint angles [deg]
//   p x y z pitch roll      move to pose via IK [cm, deg]
//   h                       move to HOME
//   o / c                   open / close claw
//   s                       print state
//   [x, y, z]               list IK solutions for the top block there
//   k                       pick it up with solution k, stack at drop-off
//   r                       reset tower count (after clearing the tower)
//   x                       stop motion and sequence
//   t ch angle              raw servo test: one channel, 0-225 deg
//   ?                       help
//
// Move commands are refused while the arm is moving (send x first),
// and stop a running sequence.
// ============================================================

// Read up to n numbers following the command word. Returns how many were
// read, or -1 if a token is not a valid finite number (so "j a b c d e" is
// rejected instead of being read as zeros).
int readNumbers(float* out, int n) {
    int k = 0;
    char* tok;
    while (k < n && (tok = strtok(NULL, " ,")) != NULL) {
        char* end;
        float val = strtod(tok, &end);
        if (end == tok || *end != '\0' || !isfinite(val)) return -1;
        out[k++] = val;
    }
    return k;
}

// Motion commands are refused mid-move: restarting a profile from rest
// while the arm is moving would cause a jerk.
bool busy() {
    if (!robot.isMoving()) return false;
    Serial.println(F("busy - wait, or send x to stop"));
    return true;
}

void printHelp() {
    Serial.println(F("j q1 q2 q3 q4 q5    joint move [deg]"));
    Serial.println(F("p x y z pitch roll  pose move [cm, deg]; pitch 0 = down, 90 = out"));
    Serial.println(F("h                   home"));
    Serial.println(F("o / c               open / close claw"));
    Serial.println(F("s                   state"));
    Serial.println(F("[x, y, z]           list IK solutions for a block (paper cm)"));
    Serial.println(F("k                   pick it up with solution k"));
    Serial.println(F("r                   reset tower count"));
    Serial.println(F("x                   stop"));
    Serial.println(F("t ch angle          raw servo test (0-225 deg on one channel)"));
}

// Lines starting with a number or '[': "[x, y, z]" or "k".
void handleNumbers(char* line) {
    float v[3];
    int n = 0;
    for (char* tok = strtok(line, " ,[]"); tok != NULL; tok = strtok(NULL, " ,[]")) {
        char* end;
        float val = strtod(tok, &end);
        if (end == tok || *end != '\0' || !isfinite(val) || n == 3) { n = -1; break; }
        v[n++] = val;
    }
    if (n == 3) listSolutions(v[0], v[1], v[2]);
    else if (n == 1) { if (!busy()) runSolution((int)v[0]); }
    else Serial.println(F("type [x, y, z] for a block, or a solution number"));
}

void handleCommand(char* line) {
    while (*line == ' ') line++;
    if (strchr("0123456789-+.[", *line) && *line) { handleNumbers(line); return; }

    char* cmd = strtok(line, " ,");
    if (cmd == NULL) return;

    float v[NUM_JOINTS];

    if (!strcmp(cmd, "j")) {
        if (readNumbers(v, 5) < 5) { Serial.println(F("usage: j q1 q2 q3 q4 q5")); return; }
        if (busy()) return;
        runSequence = false;
        robot.moveTo(v);
    }
    else if (!strcmp(cmd, "p")) {
        if (readNumbers(v, 5) < 5) { Serial.println(F("usage: p x y z pitch roll")); return; }
        if (busy()) return;
        runSequence = false;
        if (!robot.moveToPose(v[0], v[1], v[2], v[3], v[4]))
            Serial.println(F("unreachable"));
    }
    else if (!strcmp(cmd, "h"))   { if (busy()) return; runSequence = false; robot.moveTo(HOME); }
    else if (!strcmp(cmd, "o"))   robot.openClaw();
    else if (!strcmp(cmd, "c"))   robot.closeClaw();
    else if (!strcmp(cmd, "s"))   robot.printState();
    else if (!strcmp(cmd, "r"))   { stackCount = 0; Serial.println(F("tower count reset")); }
    else if (!strcmp(cmd, "x"))   { runSequence = false; robot.stop(); Serial.println(F("stopped")); }
    else if (!strcmp(cmd, "t")) {     // raw servo test, bypasses joint config
        if (readNumbers(v, 2) < 2 || v[0] < 0 || v[0] > 15) {
            Serial.println(F("usage: t channel(0-15) servoAngle(0-225)"));
            return;
        }
        runSequence = false;
        robot.stop();
        writeServo((int)v[0], v[1], 120, 500);
        Serial.println(F("pose no longer tracked - send h before j/p"));
    }
    else if (!strcmp(cmd, "?"))   printHelp();
    else Serial.println(F("unknown command, type ? for help"));
}

// Collect characters into a line without blocking; run it on newline.
void readSerial() {
    static char line[64];
    static int len = 0;
    static bool tooLong = false;

    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (tooLong) Serial.println(F("line too long - ignored"));
            else if (len > 0) {
                line[len] = '\0';
                handleCommand(line);
            }
            len = 0;
            tooLong = false;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = ch;
        } else {
            tooLong = true;
        }
    }
}

// ============================================================
// Main
// ============================================================

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(1000);

    // Warn if a joint's configured limits exceed what its servo can reach.
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (jointMin(i) > JOINTS[i].minAngle || jointMax(i) < JOINTS[i].maxAngle) {
            Serial.print(F("WARNING: J")); Serial.print(i + 1);
            Serial.print(F(" limits narrowed to ")); Serial.print(jointMin(i));
            Serial.print(F(" .. ")); Serial.print(jointMax(i));
            Serial.println(F(" (servo range) - check offset/limits"));
        }
    }

    robot.openClaw();
    robot.setJointAngles(HOME);
    delay(1500);
    robot.printState();
    Serial.println(F("Ready. Type ? for commands."));
}

void loop() {
    static bool wasMoving = false;

    readSerial();
    robot.update();

    if (runSequence) sequenceStep();
    else if (wasMoving && !robot.isMoving()) robot.printState();   // manual move done

    wasMoving = robot.isMoving();
}
