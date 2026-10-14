/*
 * ============================================================================
 *  Intro to Robotics — Robot Demo 3: Inverse Kinematics pick & place
 *  Hardware : Arduino Mega 2560 + PCA9685 16-ch servo driver + MG996R servos
 *  Library  : "Adafruit PWM Servo Driver Library" (Library Manager)
 *
 *  Wiring
 *    PCA9685 VCC -> Mega 5V        PCA9685 GND -> Mega GND
 *    PCA9685 SDA -> Mega pin 20    PCA9685 SCL -> Mega pin 21
 *    PCA9685 V+ (green screw terminal) -> separate 5-6 V supply, >= 5 A
 *    (Never power the MG996Rs from the Mega's 5 V pin.)
 *
 *  Serial Monitor: 115200 baud, line ending "Newline"
 *    5 10 0        -> top block at x=5 cm, y=10 cm, z=0 cm. Prints the IK solutions
 *                     and asks which one to use.  [5,10,0] also works.
 *    2             -> (after a target) use solution #2 and do the pick & place
 *    5 10 0 2      -> target and solution in one line
 *    r             -> re-run the last target (pick a different solution)
 *    v             -> visit every valid solution at the last target (no gripping)
 *    h             -> home          o / c -> open / close gripper
 *    t             -> tower height  z -> reset tower to 0 blocks
 *    j N DEG       -> raw servo angle on joint N (1..6) — for calibration
 *    d A1 A2 A3 A4 -> move to DH angles th1..th4 (deg) and print the FK tool point
 *    g             -> toggle streaming the trapezoidal profile (pos, vel) for the Serial Plotter
 *    p             -> print current servo angles
 *
 *  Every motion uses a trapezoidal velocity profile (constant accel, cruise,
 *  constant decel): joint moves in deg/s, vertical approach/retreat in mm/s.
 *    ?             -> help
 *
 *  z of a block = height of its BOTTOM face in cm (a block on the paper is z=0,
 *  the 2nd block of a stack is z = BLOCK_MM/10, etc.).
 * ============================================================================
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>
#include <ctype.h>
#include "ik.h"

// ============================================================================
// 1. ARM GEOMETRY — DH parameters from HW3 (mm)
//    d5 = distance from the wrist-pitch axis to the point between the finger
//    pads where the block is held. Measure yours and adjust.
// ============================================================================
const ik::ArmGeom ARM = { 110.0f, 104.0f, 99.0f, 168.0f };   // d1, r2, r3, d5

// ============================================================================
// 2. WORKSPACE / TASK
// ============================================================================
const float BASE_X_CM      = 0.0f;  // where the base axis z0 hits the graph paper (cm)
const float BASE_Y_CM      = -1.1f;
const float BASE_YAW_DEG   = 0.0f;  // angle of the robot's x0 from the paper's +x axis
const float BASE_HEIGHT_MM = 0.0f;  // height of the frame-0 origin above the paper
                                    // (d1 must be measured from that same origin)

const float BLOCK_MM       = 25.0f; // block edge length
const float GRASP_MM       = 15.0f; // grip height above the block's bottom face
const float PLACE_X_CM     = 10.0f, PLACE_Y_CM = 8.0f, PLACE_Z_CM = 0.2f;  // drop-off in cm
const float RELEASE_GAP_MM = 3.0f;  // let go this far above the resting height
const float LIFT_MM        = 60.0f; // approach / retreat height above a block
const float MIN_GRIP_Z_MM  = 8.0f;  // never drive the grip point below this (paper = 0)

const bool    TOWER_MODE = true;    // extra credit: stack placed blocks into a tower
const uint8_t TOWER_MAX  = 3;

// ============================================================================
// 3. SERVO CALIBRATION — the part you MUST tune on your robot
//    servoDeg = zero + dir * thetaDH   (degrees)
//    zero : servo angle when the DH joint angle is 0
//    dir  : +1 or -1 if the servo turns the opposite way to the DH angle
//    lo/hi: allowed servo range (degrees). Solutions outside it are rejected.
//    Use the "j" and "d" commands to find these values (see notes at bottom).
// ============================================================================
struct Joint { const char* name; uint8_t ch; float zero; float dir; float lo; float hi; };
enum { BASE = 0, SHOULDER, ELBOW, WRIST_P, WRIST_R, GRIP };
 
Joint J[6] = {
  // name        ch  zero  dir  lo  hi
  { "base",      0,  84,   +1.08f,   0,  180 },   // th1 = 0  -> arm points along x0
  { "shoulder",  1,   5,   +1.15f,   0,  180 },   // th2 = 0  -> upper arm horizontal, 90 = vertical
  { "elbow",     2,  90,   -1.1f,   0,  180 },   // th3 = 0  -> forearm in line with upper arm
  { "wrist",     3,  90,   -1.15f,   0,  180 },   // th4 = 0  -> gripper perpendicular to forearm
  { "roll",      4,  85,   +1.1f,    0,  180 },   // th5 = 0  -> jaws in their neutral orientation
  { "gripper",   5,   90,   -1.0f,    0,  180 },
};

const float GRIP_OPEN_DEG   = 40.0f;   // servo angle with jaws open wider than a block
const float GRIP_CLOSED_DEG = 105.0f;  // servo angle that just squeezes a block
const bool  ALIGN_ROLL      = true;    // turn the wrist so the jaws stay square to the grid


const int   SERVO_US_MIN = 620;        // pulse width at 0 deg   (MG996R ~500-2500 us)
const int   SERVO_US_MAX = 2400;       // pulse width at 180 deg
// ---- Trapezoidal velocity profile ("trapezoidal input shaping") ----
// Every move accelerates at A_MAX, cruises at V_MAX, decelerates at A_MAX.
// Short moves that never reach V_MAX become a triangle profile.
// MG996R no-load speed is ~350 deg/s at 4.8 V, so these leave plenty of margin.
const float JOINT_VMAX = 120.0f;   // deg/s   for the joint that moves the farthest
const float JOINT_AMAX = 300.0f;   // deg/s^2 (the other joints are scaled to finish together)
const float GRIP_VMAX  = 150.0f;   // deg/s   gripper open/close
const float GRIP_AMAX  = 600.0f;   // deg/s^2
const float LINE_VMAX  = 60.0f;    // mm/s    straight vertical approach / retreat
const float LINE_AMAX  = 150.0f;   // mm/s^2
const uint16_t TICK_MS = 20;       // update period = one 50 Hz servo frame

const float HOME_DH[4] = { 0, 110, -90, 0};   // upright "ready" pose (degrees)

// ----------------------------------------------------------------------------
// Types
// ----------------------------------------------------------------------------
struct Plan { float grip[3]; float hover[3]; float pitch; uint8_t idx; };

struct Trap {          // one trapezoidal profile over a distance D
  float D;             // total distance (deg or mm)
  float V;             // peak velocity actually reached
  float A;             // acceleration
  float ta;            // time spent accelerating (= time decelerating)
  float T;             // total duration (s)
};

bool plotProfile = false;   // "g" toggles streaming the profile to the Serial Plotter

// ============================================================================
Adafruit_PWMServoDriver pca(0x40);
float cur[6];                       // current commanded servo angles (deg)

// last analysed target
bool  havePending = false;
float tgtCm[3];
float pickGrip[3];
float pickPitch;
ik::Solution sols[4];
uint8_t towerCount = 0;

const char* SOL_NAME[4] = { "front, elbow up  ", "front, elbow down",
                            "back,  elbow up  ", "back,  elbow down" };

// ---------------------------------------------------------------------------
// Servo helpers
// ---------------------------------------------------------------------------
void writeServo(uint8_t j, float deg) {
  deg = constrain(deg, J[j].lo, J[j].hi);
  float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / 180.0f;
  uint16_t ticks = (uint16_t)(us * 4096.0f / 20000.0f + 0.5f);   // 50 Hz frame
  pca.setPWM(J[j].ch, 0, ticks);
  cur[j] = deg;
}

float dhToServo(uint8_t j, float thRad) { return J[j].zero + J[j].dir * thRad * 180.0f / M_PI; }

float rollDHdeg(float th1) {
  if (!ALIGN_ROLL) return 0.0f;
  float r = th1 * 180.0f / M_PI + BASE_YAW_DEG;   // +th1 (was -th1), plus the paper yaw
  while (r > 45.0f)   r -= 90.0f;
  while (r <= -45.0f) r += 90.0f;
  return r;
}

float rollServoFor(float th1) { return J[WRIST_R].zero + J[WRIST_R].dir * rollDHdeg(th1); }

// Is every joint of this solution inside its servo range? Reports the first offender.
bool withinLimits(const ik::Solution& s, const char** why) {
  if (!s.reachable) { if (why) *why = "out of reach"; return false; }
  for (uint8_t j = 0; j < 4; j++) {
    float d = dhToServo(j, s.th[j]);
    if (d < J[j].lo - 0.5f || d > J[j].hi + 0.5f) { if (why) *why = J[j].name; return false; }  // 0.5 deg rounding slack
  }
  return true;
}

void poseFrom(const ik::Solution& s, float gripDeg, float out[6]) {
  for (uint8_t j = 0; j < 4; j++) out[j] = dhToServo(j, s.th[j]);
  out[WRIST_R] = rollServoFor(s.th[0]);
  out[GRIP] = gripDeg;
}

// ---------------------------------------------------------------------------
// Trapezoidal velocity profile
//
//   v ^      ______________
//     |     /              \          accel   : 0 <= t < ta        v = A t
//     |    /                \         cruise  : ta <= t <= T-ta    v = V
//     |   /                  \        decel   : T-ta < t <= T      v = A (T-t)
//     +--+----+--------+------+--> t
//        0    ta     T-ta     T
//
// If D < Vmax^2/A the cruise phase vanishes:  ta = sqrt(D/A), V = A ta, T = 2 ta.
// Otherwise:                                  ta = Vmax/A,    V = Vmax, T = D/Vmax + ta.
// ---------------------------------------------------------------------------
Trap makeTrap(float D, float vmax, float amax) {
  Trap p;
  p.D = D; p.A = amax;
  if (D * amax <= vmax * vmax) { p.ta = sqrtf(D / amax); p.V = amax * p.ta; p.T = 2.0f * p.ta; }
  else                         { p.V = vmax; p.ta = vmax / amax; p.T = D / vmax + p.ta; }
  return p;
}

float trapPos(const Trap& p, float t) {          // distance covered at time t
  if (t <= 0)          return 0;
  if (t >= p.T)        return p.D;
  if (t < p.ta)        return 0.5f * p.A * t * t;
  if (t <= p.T - p.ta) return 0.5f * p.A * p.ta * p.ta + p.V * (t - p.ta);
  float r = p.T - t;   return p.D - 0.5f * p.A * r * r;
}

float trapVel(const Trap& p, float t) {          // speed at time t
  if (t <= 0 || t >= p.T) return 0;
  if (t < p.ta)           return p.A * t;
  if (t <= p.T - p.ta)    return p.V;
  return p.A * (p.T - t);
}

// Paces a profile in real time: returns the path fraction s (0..1) once per
// servo frame and false after the final s = 1 sample has been delivered.
struct TrapClock {
  Trap p; unsigned long t0, nextTick; bool done; float s;
  void start(const Trap& prof) { p = prof; t0 = millis(); nextTick = t0; done = false; s = 0; }
  bool next() {
    if (done) return false;
    while ((long)(millis() - nextTick) < 0) { }      // wait for the next frame
    nextTick += TICK_MS;
    float t = (millis() - t0) / 1000.0f;
    if (t >= p.T || p.D <= 0) { t = p.T; done = true; }
    s = (p.D > 0) ? trapPos(p, t) / p.D : 1.0f;
    if (plotProfile) {   // Serial Plotter: position and velocity of the leading axis
      Serial.print(F("pos:")); Serial.print(s * p.D, 2);
      Serial.print(F("\tvel:")); Serial.println(trapVel(p, t), 2);
    }
    return true;
  }
};

// Joint-space move. The joint with the largest displacement follows the
// trapezoid; every other joint is scaled by the same s, so all start and
// stop together and every joint's velocity is also trapezoidal.
void moveJoints(const float tgt[6], float vmax, float amax) {
  float start[6], D = 0;
  for (uint8_t j = 0; j < 6; j++) { start[j] = cur[j]; D = max(D, (float)fabs(tgt[j] - start[j])); }
  if (D < 0.05f) return;
  TrapClock clk; clk.start(makeTrap(D, vmax, amax));
  while (clk.next())
    for (uint8_t j = 0; j < 6; j++) writeServo(j, start[j] + (tgt[j] - start[j]) * clk.s);
}

void moveSmooth(const float tgt[6]) { moveJoints(tgt, JOINT_VMAX, JOINT_AMAX); }

void setGripper(float deg) {
  float t[6]; memcpy(t, cur, sizeof(t));
  t[GRIP] = deg;
  moveJoints(t, GRIP_VMAX, GRIP_AMAX);
  delay(250);                       // let the jaws settle on the block
}

void goHome() {
  ik::Solution s; s.reachable = true;
  for (uint8_t j = 0; j < 4; j++) s.th[j] = HOME_DH[j] * M_PI / 180.0f;
  float t[6]; poseFrom(s, cur[GRIP], t);
  moveSmooth(t);
}

// ---------------------------------------------------------------------------
// Coordinates & IK wrappers
// ---------------------------------------------------------------------------
// paper (cm) -> robot frame 0 (mm)
void paperToBase(float xcm, float ycm, float zcm, float out[3]) {
  float dx = (xcm - BASE_X_CM) * 10.0f, dy = (ycm - BASE_Y_CM) * 10.0f;
  float a = -BASE_YAW_DEG * M_PI / 180.0f;
  out[0] = cosf(a) * dx - sinf(a) * dy;
  out[1] = sinf(a) * dx + cosf(a) * dy;
  out[2] = zcm * 10.0f - BASE_HEIGHT_MM;
}

bool solveIdx(const float p[3], float pitch, uint8_t idx, ik::Solution& s) {
  ik::Solution all[4];
  ik::solve(ARM, p[0], p[1], p[2], pitch, all);
  s = all[idx];
  return withinLimits(s, NULL);
}

// Gripper pitch to try: straight down first, then tilt up to 45 deg either way.
float pitchCandidate(uint8_t k) {        // k = 0..18
  int dev = ((k + 1) / 2) * 5 * ((k % 2) ? 1 : -1);
  return (-90.0f + dev) * M_PI / 180.0f;
}
const uint8_t N_PITCH = 19;

// Straight Cartesian line a -> b on IK branch idx.
// execute=false: only checks that every 4 mm sample is solvable and inside joint limits.
// execute=true : first moves (joint space) to a, then slides along the line with a
//                trapezoidal speed profile in mm/s, solving IK every servo frame.
bool lineMove(const float a[3], const float b[3], float pitch, uint8_t idx, bool execute) {
  float L = 0; for (uint8_t i = 0; i < 3; i++) L += (b[i] - a[i]) * (b[i] - a[i]);
  L = sqrtf(L);
  int n = max(1, (int)ceil(L / 4.0f));
  ik::Solution s;
  for (int i = 0; i <= n; i++) {                       // feasibility check
    float p[3]; for (uint8_t k = 0; k < 3; k++) p[k] = a[k] + (b[k] - a[k]) * i / n;
    if (!solveIdx(p, pitch, idx, s)) return false;
  }
  if (!execute) return true;

  float t[6];
  solveIdx(a, pitch, idx, s);
  poseFrom(s, cur[GRIP], t);
  moveSmooth(t);                                       // get onto the start of the line
  if (L < 0.5f) return true;

  TrapClock clk; clk.start(makeTrap(L, LINE_VMAX, LINE_AMAX));
  while (clk.next()) {
    float p[3]; for (uint8_t k = 0; k < 3; k++) p[k] = a[k] + (b[k] - a[k]) * clk.s;
    if (solveIdx(p, pitch, idx, s)) {
      poseFrom(s, cur[GRIP], t);
      for (uint8_t j = 0; j < 5; j++) writeServo(j, t[j]);
    }
  }
  return true;
}

// Build approach/retreat for one IK branch; shrinks the lift height if needed.
bool makePlan(const float grip[3], float pitch, uint8_t idx, Plan& P) {
  ik::Solution s;
  if (!solveIdx(grip, pitch, idx, s)) return false;
  for (float lift = LIFT_MM; lift >= 15.0f; lift -= 5.0f) {
    float hover[3] = { grip[0], grip[1], grip[2] + lift };
    if (lineMove(hover, grip, pitch, idx, false)) {
      memcpy(P.grip, grip, sizeof(P.grip)); memcpy(P.hover, hover, sizeof(P.hover));
      P.pitch = pitch; P.idx = idx;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------
void printAngles(const ik::Solution& s) {
  for (uint8_t j = 0; j < 4; j++) {
    Serial.print(F("  th")); Serial.print(j + 1); Serial.print('=');
    float d = s.th[j] * 180.0f / M_PI;
    if (d >= 0) Serial.print(' ');
    if (fabs(d) < 100) Serial.print(' ');
    if (fabs(d) < 10) Serial.print(' ');
    Serial.print(d, 1);
  }
}

void help() {
  Serial.println(F("\n--- Robot Demo 3: IK pick & place ---"));
  Serial.println(F(" x y z       top block location in cm (e.g. 8 -6 2.5 or [8,-6,2.5])"));
  Serial.println(F(" N           choose IK solution N (1-4) for the last target"));
  Serial.println(F(" x y z N     target + solution in one line"));
  Serial.println(F(" r           re-run last target        v  visit all valid solutions"));
  Serial.println(F(" h home   o open   c close   t tower height   z reset tower"));
  Serial.println(F(" j N DEG     raw servo angle on joint N (1 base .. 6 gripper)"));
  Serial.println(F(" d a1 a2 a3 a4   go to DH angles (deg), prints FK"));
  Serial.println(F(" g           toggle trapezoid profile output for the Serial Plotter"));
  Serial.println(F(" p           print servo angles        ?  this help\n"));
}

// ---------------------------------------------------------------------------
// Analyse a target: count & list the IK solutions
// ---------------------------------------------------------------------------
void analyze(float x, float y, float z) {
  tgtCm[0] = x; tgtCm[1] = y; tgtCm[2] = z;
  paperToBase(x, y, z, pickGrip);
  pickGrip[2] += GRASP_MM;
  havePending = false;

  Serial.print(F("\nTarget block [")); Serial.print(x, 1); Serial.print(','); Serial.print(y, 1);
  Serial.print(','); Serial.print(z, 1); Serial.println(F("] cm"));

  if (pickGrip[2] < MIN_GRIP_Z_MM) { Serial.println(F("  Grip point would be below the table. Rejected.")); return; }

  // Choose the gripper pitch: straight down if any solution works, else tilt.
  int nGeo = 0, nOk = 0;
  for (uint8_t k = 0; k < N_PITCH; k++) {
    float p = pitchCandidate(k);
    ik::Solution tmp[4];
    int g = ik::solve(ARM, pickGrip[0], pickGrip[1], pickGrip[2], p, tmp);
    int ok = 0; for (uint8_t i = 0; i < 4; i++) if (withinLimits(tmp[i], NULL)) ok++;
    if (k == 0 || ok > 0) { memcpy(sols, tmp, sizeof(sols)); pickPitch = p; nGeo = g; nOk = ok; }
    if (ok > 0) break;
  }

  Serial.print(F("  Gripper pitch: ")); Serial.print(pickPitch * 180.0f / M_PI, 0);
  Serial.println(F(" deg (-90 = straight down)"));
  for (uint8_t i = 0; i < 4; i++) {
    const char* why = NULL;
    bool ok = withinLimits(sols[i], &why);
    Serial.print(F("  #")); Serial.print(i + 1); Serial.print(' '); Serial.print(SOL_NAME[i]);
    if (sols[i].reachable) printAngles(sols[i]);
    if (ok) Serial.println(F("   OK"));
    else if (!sols[i].reachable) Serial.println(F("   unreachable"));
    else { Serial.print(F("   X ")); Serial.print(why); Serial.println(F(" limit")); }
  }
  Serial.print(F(">> IK solutions detected: ")); Serial.print(nGeo);
  Serial.print(F("  (")); Serial.print(nOk); Serial.println(F(" within joint limits)"));

  if (nOk == 0) { Serial.println(F("  No usable solution for this target.")); return; }
  havePending = true;
  Serial.println(F("  Type the solution number to use (1-4):"));
}

// ---------------------------------------------------------------------------
// Pick & place with the chosen solution
// ---------------------------------------------------------------------------
bool findPlacePlan(uint8_t preferIdx, Plan& P) {
  float g[3];
  paperToBase(PLACE_X_CM, PLACE_Y_CM, PLACE_Z_CM, g);
  if (TOWER_MODE) g[2] += towerCount * BLOCK_MM;
  g[2] += GRASP_MM + RELEASE_GAP_MM;
  for (uint8_t k = 0; k < N_PITCH; k++) {
    float p = pitchCandidate(k);
    if (makePlan(g, p, preferIdx, P)) return true;          // same branch first
    for (uint8_t i = 0; i < 4; i++) if (i != preferIdx && makePlan(g, p, i, P)) return true;
  }
  return false;
}

void pickAndPlace(uint8_t idx) {
  if (!havePending) { Serial.println(F("Enter a target first (x y z).")); return; }
  if (idx > 3 || !withinLimits(sols[idx], NULL)) {
    Serial.print(F("Solution #")); Serial.print(idx + 1); Serial.println(F(" is not valid for this target."));
    return;
  }
  if (TOWER_MODE && towerCount >= TOWER_MAX) {
    Serial.println(F("Tower is full (3 blocks). Clear it and type z to reset."));
    return;
  }
  Plan pick, place;
  if (!makePlan(pickGrip, pickPitch, idx, pick)) {
    Serial.println(F("This solution can't approach straight down from above. Pick another.")); return;
  }
  if (!findPlacePlan(idx, place)) { Serial.println(F("Drop-off location is unreachable. Check BASE_X/Y.")); return; }

  Serial.print(F("Using solution #")); Serial.print(idx + 1); Serial.print(F(" (")); Serial.print(SOL_NAME[idx]); Serial.println(F(")"));
  if (place.idx != idx) { Serial.print(F("  drop-off uses solution #")); Serial.println(place.idx + 1); }

  // --- pick ---
  setGripper(GRIP_OPEN_DEG);
  lineMove(pick.hover, pick.hover, pick.pitch, idx, true);   // travel to hover
  lineMove(pick.hover, pick.grip,  pick.pitch, idx, true);   // straight down
  Serial.println(F("  grasp"));
  setGripper(GRIP_CLOSED_DEG);
  lineMove(pick.grip, pick.hover, pick.pitch, idx, true);    // straight up

  // --- carry --- (go through home if the base would swing a long way or branch changes)
  ik::Solution sA, sB;
  solveIdx(pick.hover, pick.pitch, idx, sA);
  solveIdx(place.hover, place.pitch, place.idx, sB);
  if (sA.back != sB.back || fabs(ik::wrapPi(sA.th[0] - sB.th[0])) > M_PI / 2) goHome();

  // --- place ---
  lineMove(place.hover, place.hover, place.pitch, place.idx, true);
  lineMove(place.hover, place.grip,  place.pitch, place.idx, true);
  Serial.println(F("  release"));
  setGripper(GRIP_OPEN_DEG);
  lineMove(place.grip, place.hover, place.pitch, place.idx, true);
  goHome();

  if (TOWER_MODE) {
    towerCount++;
    Serial.print(F("Done. Tower at [10,8] is now ")); Serial.print(towerCount); Serial.println(F(" block(s) high."));
  } else Serial.println(F("Done."));
  Serial.println(F("Type r to re-run this position with another solution, or a new x y z."));
}

// Visit every valid solution at the last target (shows them without grabbing).
void visitAll() {
  if (!havePending) { Serial.println(F("Enter a target first.")); return; }
  setGripper(GRIP_OPEN_DEG);
  for (uint8_t i = 0; i < 4; i++) {
    Plan P;
    if (!withinLimits(sols[i], NULL) || !makePlan(pickGrip, pickPitch, i, P)) continue;
    Serial.print(F("  showing #")); Serial.println(i + 1);
    goHome();
    lineMove(P.hover, P.hover, P.pitch, i, true);
    lineMove(P.hover, P.grip, P.pitch, i, true);
    delay(1500);
    lineMove(P.grip, P.hover, P.pitch, i, true);
  }
  goHome();
}

// ---------------------------------------------------------------------------
// Command parsing
// ---------------------------------------------------------------------------
int parseNumbers(char* s, float* v, int maxN) {
  for (char* c = s; *c; c++) if (*c == '[' || *c == ']' || *c == ',' || *c == '(' || *c == ')') *c = ' ';
  int n = 0; char* end;
  while (n < maxN) {
    while (*s == ' ' || *s == '\t') s++;
    if (!*s) break;
    float x = strtod(s, &end);
    if (end == s) break;
    v[n++] = x; s = end;
  }
  return n;
}

void handleLine(char* line) {
  while (*line == ' ') line++;
  if (!*line) return;
  char c = tolower(line[0]);
  float v[5];

  if (isalpha(c) || c == '?') {
    int n = parseNumbers(line + 1, v, 5);
    switch (c) {
      case 'h': goHome(); break;
      case 'o': setGripper(GRIP_OPEN_DEG); break;
      case 'c': setGripper(GRIP_CLOSED_DEG); break;
      case 'r':
        if (tgtCm[2] > -1.0f) analyze(tgtCm[0], tgtCm[1], tgtCm[2]);
        else Serial.println(F("No previous target."));
        break;
      case 'v': visitAll(); break;
      case 'z': towerCount = 0; Serial.println(F("Tower reset to 0.")); break;
      case 'g':
        plotProfile = !plotProfile;
        Serial.println(plotProfile ? F("Profile plotting ON (open Tools > Serial Plotter)")
                                   : F("Profile plotting OFF"));
        break;
      case 't': Serial.print(F("Tower height: ")); Serial.print(towerCount); Serial.println(F(" block(s)")); break;
      case 'j':
        if (n == 2 && v[0] >= 1 && v[0] <= 6) {
          uint8_t j = (uint8_t)v[0] - 1;
          float t[6]; memcpy(t, cur, sizeof(t)); t[j] = v[1];
          moveSmooth(t);
          Serial.print(J[j].name); Serial.print(F(" servo = ")); Serial.println(cur[j], 1);
        } else Serial.println(F("usage: j N DEG   (N = 1..6)"));
        break;
      case 'd':
        if (n == 4) {
          ik::Solution s; s.reachable = true;
          for (uint8_t j = 0; j < 4; j++) s.th[j] = v[j] * M_PI / 180.0f;
          const char* why;
          if (!withinLimits(s, &why)) { Serial.print(F("Outside ")); Serial.print(why); Serial.println(F(" limit.")); break; }
          float o[3]; ik::fk(ARM, s.th, o);
          if (o[2] + BASE_HEIGHT_MM < MIN_GRIP_Z_MM) { Serial.println(F("That pose puts the gripper into the table. Refused.")); break; }
          float t[6]; poseFrom(s, cur[GRIP], t); moveSmooth(t);
          Serial.print(F("FK tool point (frame 0, mm): ")); Serial.print(o[0], 1); Serial.print(' ');
          Serial.print(o[1], 1); Serial.print(' '); Serial.println(o[2], 1);
        } else Serial.println(F("usage: d th1 th2 th3 th4   (degrees)"));
        break;
      case 'p':
        for (uint8_t j = 0; j < 6; j++) { Serial.print(J[j].name); Serial.print('='); Serial.print(cur[j], 1); Serial.print(F("  ")); }
        Serial.println();
        break;
      default: help();
    }
    return;
  }

  int n = parseNumbers(line, v, 5);
  if (n == 1) {
    if (!havePending) { Serial.println(F("Enter a target first: x y z (cm)")); return; }
    pickAndPlace((uint8_t)(v[0] + 0.5f) - 1);
  } else if (n == 3 || n == 4) {
    analyze(v[0], v[1], v[2]);
    if (n == 4 && havePending) pickAndPlace((uint8_t)(v[3] + 0.5f) - 1);
  } else {
    help();
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(10);

  tgtCm[2] = -1;  // no target yet
  // Servos have no position feedback, so write the home pose directly at power-up.
  ik::Solution s; s.reachable = true;
  for (uint8_t j = 0; j < 4; j++) s.th[j] = HOME_DH[j] * M_PI / 180.0f;
  float t[6]; poseFrom(s, GRIP_OPEN_DEG, t);
  for (uint8_t j = 0; j < 6; j++) { writeServo(j, t[j]); delay(150); }

  help();
}

char buf[64];
uint8_t len = 0;

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

/* ============================================================================
 *  CALIBRATION CHECKLIST (do this once, before recording)
 *  1. Remove the horns or keep the arm clear. Type  j 1 90 ... j 6 90  to centre.
 *  2. Base:     find the servo angle where the arm points along the paper's +x.
 *               That angle is J[BASE].zero. If increasing the angle turns the arm
 *               toward +y it's dir=+1, otherwise dir=-1.
 *  3. Shoulder: find the angle where the upper arm is horizontal (th2=0) and check
 *               which way is "up" (th2 positive = lifts the arm).
 *  4. Elbow:    angle where the forearm is in line with the upper arm (th3=0);
 *               positive th3 bends it upward.
 *  5. Wrist:    angle where the gripper is perpendicular to the forearm (th4=0);
 *               positive th4 rotates it the same way as th2/th3.
 *  6. Verify with  dn 0 90 0 0    (arm straight up, gripper pointing forward) and
 *               d 0 90 -90 0  (upper arm up, forearm level, gripper poiting down).
 *               The printed FK point should match a ruler measurement.
 *  7. Gripper:  find GRIP_OPEN_DEG and GRIP_CLOSED_DEG with  j 6 DEG.
 *  8. Measure where the base axis sits on the graph paper -> BASE_X_CM / BASE_Y_CM.
 * ============================================================================ */
