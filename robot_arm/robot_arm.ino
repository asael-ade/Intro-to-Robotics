// 5-DOF robot arm + claw on a PCA9685, with synchronized trapezoidal moves.
// Units: degrees, cm, seconds.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

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
    { 0,  -90,    90,    90,   false, 120, 500,  60, 120 },
    { 1,    0,   135,    20,   false, 120, 500,  45,  80 },
    { 3,  -90,    90,    90,    true, 120, 500,  60, 120 },
    { 3,  -90,    90,    90,   false, 120, 500,  90, 180 },
    { 4,  -90,    90,    90,   false, 120, 500,  90, 180 },
};

// Claw: channel, open angle, closed angle, min pulse, max pulse
// NOTE: channel 5 is also used by joint 1 - set to the claw's real channel.
const int   CLAW_CHANNEL = 5;
const float CLAW_OPEN = 90, CLAW_CLOSED = 20;
const int   CLAW_MIN_PULSE = 120, CLAW_MAX_PULSE = 500;

// DH table: r, alpha (rad), d
const float DH[NUM_JOINTS][3] = {
    {  0.0f, PI / 2, 11.5f },
    { 10.5f, 0,       0.0f },
    {  9.5f, 0,       0.0f },
    {   0.0, PI / 2,  0.0f },
    {  0.0f, 0,       9.0f },
};

const float HOME[NUM_JOINTS] = { 0, 90, 0, -10, -10 };

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
            target_[i] = constrain(target[i], JOINTS[i].minAngle, JOINTS[i].maxAngle);
            float D = fabs(target_[i] - start[i]);
            if (D > 0.05f) {
                v = min(v, JOINTS[i].maxVel / D);
                a = min(a, JOINTS[i].maxAcc / D);
            }
        }

        if (isinf(v)) return;                   // already there

        profile.plan(v, a);
        startUs = micros();
        lastUs  = startUs - UPDATE_PERIOD_US;   // first step immediately
        moving  = true;
    }

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
    //   pitch   : theta2 + theta3 + theta4 = angle of link 4's x-axis above
    //             horizontal, in the arm's vertical plane [deg]
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
                if (cand[i] < JOINTS[i].minAngle - 0.01f ||
                    cand[i] > JOINTS[i].maxAngle + 0.01f) ok = false;
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
        Matrix<4, 4> T = Identity<4, 4>();
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
        Serial.print(T(2, 3)); Serial.println(" cm");
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
            angle[i] = constrain(q[i], c.minAngle, c.maxAngle);
            float servo = c.reversed ? c.offset - angle[i] : c.offset + angle[i];
            writeServo(c.channel, servo, c.minPulse, c.maxPulse);
        }
    }
};

// ============================================================
// Waypoint sequence (Cartesian targets, solved with IK)
// ============================================================

enum ClawAction { NONE, OPEN, CLOSE };

struct Waypoint {
    float x, y, z;              // tool position [cm]
    float pitch, roll;          // [deg], see inverseKinematics()
    ClawAction claw;            // done on arrival
    unsigned long dwellMs;      // pause after arrival
};

const Waypoint WAYPOINTS[] = {
//     x     y     z    pitch roll   claw   dwell
    {  3.6,  0.0, 32.4,  80, -10,  OPEN,  1000 },  // home
    {  8.5,  4.9, 30.4,  70,   0,  NONE,   500 },  // above object
    { 11.1,  6.4, 27.8,  60,   0,  CLOSE,  800 },  // grasp
    {  6.2,  3.6, 31.8,  80,   0,  NONE,   300 },  // lift
    {  6.2, -3.6, 31.8,  80,   0,  NONE,   300 },  // carry
    { 10.2, -5.9, 28.9,  65,   0,  OPEN,   800 },  // release
};
const int NUM_WAYPOINTS = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

// ============================================================
// Main
// ============================================================

RobotArm robot;
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

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(1000);

    robot.openClaw();
    robot.setJointAngles(HOME);
    delay(1500);
    robot.printState();

    startWaypoint();
}

// move -> arrive (claw action) -> dwell -> next waypoint
void loop() {
    robot.update();
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

    wp = (wp + 1) % NUM_WAYPOINTS;
    startWaypoint();
}
