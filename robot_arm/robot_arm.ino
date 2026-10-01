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
    int   minPulse, maxPulse;   // PCA9685 counts at 0 and 180 deg
    float maxVel, maxAcc;       // deg/s, deg/s^2
};

const JointConfig JOINTS[NUM_JOINTS] = {
//   ch    min    max    offset rev   minP maxP  vel acc
    { 6,  -90,    90,     0,   false, 120, 500,  60, 120 },  // J1 base
    { 7,    0,   135,    10,   false, 120, 500,  45,  80 },  // J2 shoulder
    { 2,  -90,   120,    45,    true, 120, 500,  60, 120 },  // J3 elbow
    { 4,  -90,    90,    90,   false, 120, 500,  90, 180 },  // J4 wrist pitch
    { 3,  -90,    90,    90,   false, 120, 500,  90, 180 },  // J5 wrist roll
};

// Usable joint limits: the configured limits, narrowed to what the servo's
// 0-180 deg travel can actually reach (servo = offset +/- joint angle).
// Everything (moves, IK, FK) uses these, so the stored pose always matches
// what was sent to the servos.
float jointMin(int i) {
    const JointConfig& c = JOINTS[i];
    return max(c.minAngle, c.reversed ? c.offset - 180 : 0 - c.offset);
}
float jointMax(int i) {
    const JointConfig& c = JOINTS[i];
    return min(c.maxAngle, c.reversed ? c.offset : 180 - c.offset);
}

// Claw: channel, open angle, closed angle, min pulse, max pulse
const int   CLAW_CHANNEL = 5;
const float CLAW_OPEN = 90, CLAW_CLOSED = 20;
const int   CLAW_MIN_PULSE = 120, CLAW_MAX_PULSE = 500;

// DH table: r, alpha (rad), d
const float DH[NUM_JOINTS][3] = {
    {  0.0f, PI / 2, 11.5f },
    { 10.5f, 0,       0.0f },
    { 10.0f, 0,       0.0f },
    {  0.0f, PI / 2,  0.0f },
    {  0.0f, 0,      15.0f },
};

const float HOME[NUM_JOINTS] = { 0, 90, 40, 0, 0 };

Adafruit_PWMServoDriver pwm;

// ============================================================
// Servo output
// ============================================================

// Float math instead of map(), which rounds to whole degrees.
void writeServo(int channel, float servoAngle, int minPulse, int maxPulse) {
    servoAngle = constrain(servoAngle, 0.0f, 180.0f);
    pwm.setPWM(channel, 0, minPulse + servoAngle / 180.0f * (maxPulse - minPulse) + 0.5f);
}

// Wrap an angle to [-180, 180] deg.
float wrap180(float a) {
    while (a >  180) a -= 360;
    while (a < -180) a += 360;
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

    // Analytic inverse kinematics.
    //   x, y, z : tool position in the base frame [cm]
    //   pitch   : theta2 + theta3 + theta4 [deg]. With the DH table above the
    //             gripper points (pitch - 90) deg from horizontal:
    //             pitch 0 = straight down, pitch 90 = straight out.
    //   roll    : theta5 [deg]
    // Writes joint angles to q. Returns false if the target is out of reach or
    // violates a joint limit. Of the two elbow solutions, picks the valid one
    // closest to the current pose. Link lengths come from the DH table.
    bool inverseKinematics(float x, float y, float z, float pitch, float roll,
                           float q[NUM_JOINTS]) const {
        const float d1 = DH[0][2], a2 = DH[1][0], a3 = DH[2][0],
                    a4 = DH[3][0], d5 = DH[4][2];

        // Base: face the target. If it is behind the base (outside +-90),
        // face the opposite way and reach backward (negative radius).
        float q1 = atan2(y, x) * RAD_TO_DEG;
        float u  = sqrt(x * x + y * y);         // radial distance
        if (q1 >  90) { q1 -= 180; u = -u; }
        if (q1 < -90) { q1 += 180; u = -u; }

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

        bool found = false;
        float bestCost = INFINITY;

        for (int elbow = -1; elbow <= 1; elbow += 2) {
            float t3 = elbow * acos(c3);
            float t2 = atan2(vw, uw) - atan2(a3 * sin(t3), a2 + a3 * cos(t3));

            float cand[NUM_JOINTS];
            cand[0] = q1;
            cand[1] = wrap180(t2 * RAD_TO_DEG);
            cand[2] = wrap180(t3 * RAD_TO_DEG);
            cand[3] = wrap180(pitch - cand[1] - cand[2]);
            cand[4] = roll;

            bool ok = true;
            float cost = 0;                     // distance from current pose
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (cand[i] < jointMin(i) - 0.01f ||
                    cand[i] > jointMax(i) + 0.01f) ok = false;
                cost += fabs(cand[i] - angle[i]);
            }

            if (ok && cost < bestCost) {
                bestCost = cost;
                for (int i = 0; i < NUM_JOINTS; i++) q[i] = cand[i];
                found = true;
            }
        }
        return found;
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
// Waypoint sequence (Cartesian targets, solved with IK)
// Started with the "run" command.
// ============================================================

enum ClawAction { NONE, OPEN, CLOSE };

struct Waypoint {
    float x, y, z;              // tool position [cm]
    float pitch, roll;          // [deg], see inverseKinematics()
    ClawAction claw;            // done on arrival
    unsigned long dwellMs;      // pause after arrival
};

// Gripper pointing straight down (pitch 0) for pick and place.
// All points are reachable with the limits above (J1 only turns 0..90,
// so both sides are at positive y). Replace with your own positions.
const Waypoint WAYPOINTS[] = {
//     x     y     z    pitch roll   claw   dwell
    { 14,    6,    9,    0,   0,  OPEN,   500 },  // above object
    { 14,    6,    3,    0,   0,  CLOSE,  800 },  // grasp
    { 14,    6,    9,    0,   0,  NONE,   300 },  // lift
    {  6,   14,    9,    0,   0,  NONE,   300 },  // carry
    {  6,   14,    3,    0,   0,  OPEN,   800 },  // release
    {  6,   14,    9,    0,   0,  NONE,   300 },  // retreat
};
const int NUM_WAYPOINTS = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

RobotArm robot;

bool runSequence = false;       // true while the waypoint sequence runs
int wp = 0;
bool arrived = false;
unsigned long arrivedMs = 0;

// Start moving to waypoint wp. If IK fails, skip it: mark it as already
// "arrived" without doing its claw action, so the sequence just moves on.
void startWaypoint() {
    const Waypoint& w = WAYPOINTS[wp];
    arrived = !robot.moveToPose(w.x, w.y, w.z, w.pitch, w.roll);
    if (arrived) {
        arrivedMs = millis();
        Serial.print("Waypoint ");
        Serial.print(wp);
        Serial.println(" unreachable - skipped");
    }
}

// move -> arrive (claw action) -> dwell -> next waypoint
void sequenceStep() {
    if (robot.isMoving()) return;

    if (!arrived) {
        arrived = true;
        arrivedMs = millis();
        robot.printState();
        if (WAYPOINTS[wp].claw == OPEN)  robot.openClaw();
        if (WAYPOINTS[wp].claw == CLOSE) robot.closeClaw();
        return;
    }

    if (millis() - arrivedMs < WAYPOINTS[wp].dwellMs) return;

    if (++wp >= NUM_WAYPOINTS) {            // one pass per "run"
        runSequence = false;
        Serial.println(F("Sequence complete"));
        return;
    }
    startWaypoint();
}

// ============================================================
// Serial commands (115200 baud, newline-terminated)
//
//   j q1 q2 q3 q4 q5        move to joint angles [deg]
//   p x y z pitch roll      move to pose via IK [cm, deg]
//   h                       move to HOME
//   o / c                   open / close claw
//   s                       print state
//   run                     start waypoint sequence
//   x                       stop motion and sequence
//   t ch angle              raw servo test: one channel, 0-180 deg
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
    Serial.println(F("run                 start waypoint sequence"));
    Serial.println(F("x                   stop"));
    Serial.println(F("t ch angle          raw servo test (0-180 deg on one channel)"));
}

void handleCommand(char* line) {
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
    else if (!strcmp(cmd, "run")) { if (busy()) return; runSequence = true; wp = 0; startWaypoint(); }
    else if (!strcmp(cmd, "x"))   { runSequence = false; robot.stop(); Serial.println(F("stopped")); }
    else if (!strcmp(cmd, "t")) {     // raw servo test, bypasses joint config
        if (readNumbers(v, 2) < 2 || v[0] < 0 || v[0] > 15) {
            Serial.println(F("usage: t channel(0-15) servoAngle(0-180)"));
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