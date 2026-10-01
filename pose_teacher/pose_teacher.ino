/*
 * pose_teacher.ino
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);

const uint8_t  NUM_JOINTS    = 6;
const uint8_t  MAX_POSES     = 40;
const uint16_t SERVOMIN      = 150;
const uint16_t SERVOMAX      = 600;
const uint8_t  STEP_DELAY    = 15;   // ms per degree
const uint16_t DEFAULT_DWELL = 300;  // ms pause written into each waypoint

/* Channel map as actually wired. */
const char *JOINT_NAME[NUM_JOINTS] = {
  "wristRot", "wristPitch", "gripper", "elbow", "2", "base"
};

/* From Phase 2. Re-run the limit finder if you rewire anything. */
const uint8_t LIM_MIN[NUM_JOINTS] = {   0,   0,   5,   0,   0,   0 };
const uint8_t LIM_MAX[NUM_JOINTS] = { 159, 153,  65, 110, 130, 145 };

/* Must sit inside every limit above. Set the arm here by hand first. */
const uint8_t NEUTRAL[NUM_JOINTS] = {  80,  75,  35,  55,  65,  70 };

/* Gripper shortcuts. If 'o' closes and 'g' opens, swap these two. */
const uint8_t GRIPPER_CH     = 2;
const uint8_t GRIPPER_OPEN   = 10;
const uint8_t GRIPPER_CLOSED = 60;

uint8_t currentAngle[NUM_JOINTS];
uint8_t pose[MAX_POSES][NUM_JOINTS];
uint8_t poseCount = 0;

uint8_t selected = 0;
uint8_t stepSize = 2;
bool    engaged  = false;

/* ---------- low-level output ---------- */

void driveSafe(uint8_t ch, int16_t a) {
  a = constrain(a, LIM_MIN[ch], LIM_MAX[ch]);
  uint16_t width = map(a, 0, 180, SERVOMIN, SERVOMAX);
  uint16_t on    = (uint16_t)ch * 256;
  pwm1.setPWM(ch, on, (on + width) % 4096);
  currentAngle[ch] = (uint8_t)a;
}

void sweepTo(uint8_t ch, uint8_t target) {
  int16_t a = currentAngle[ch];
  target = constrain(target, LIM_MIN[ch], LIM_MAX[ch]);
  if (a == target) return;
  int8_t dir = (target > a) ? 1 : -1;
  while (a != (int16_t)target) {
    a += dir;
    driveSafe(ch, a);
    delay(STEP_DELAY);
  }
}

void relaxAll() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) pwm1.setPWM(i, 0, 4096);
  engaged = false;
  Serial.println(F("\n*** RELAXED - servos limp. SUPPORT THE ARM. 'e' to re-engage. ***"));
}

void engage() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    currentAngle[i] = NEUTRAL[i];
    driveSafe(i, NEUTRAL[i]);
    delay(120);
  }
  engaged = true;
  Serial.println(F("\nEngaged at NEUTRAL."));
}

/* ---------- pose capture ---------- */

void printPoseLine(uint8_t idx) {
  Serial.print(F("  {{ "));
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    if (pose[idx][j] < 100) Serial.print(F(" "));
    if (pose[idx][j] < 10)  Serial.print(F(" "));
    Serial.print(pose[idx][j]);
    if (j < NUM_JOINTS - 1) Serial.print(F(", "));
  }
  Serial.print(F(" }, "));
  Serial.print(DEFAULT_DWELL);
  Serial.print(F("},   // "));
  Serial.println(idx);
}

void capturePose() {
  if (poseCount >= MAX_POSES) {
    Serial.println(F("Pose buffer full. 'L' to dump, then reset."));
    return;
  }
  for (uint8_t j = 0; j < NUM_JOINTS; j++) pose[poseCount][j] = currentAngle[j];
  Serial.print(F("\nCaptured pose "));
  Serial.println(poseCount);
  printPoseLine(poseCount);
  poseCount++;
}

void undoPose() {
  if (poseCount == 0) { Serial.println(F("Nothing to undo.")); return; }
  poseCount--;
  Serial.print(F("Removed pose "));
  Serial.println(poseCount);
}

void listPoses() {
  if (poseCount == 0) { Serial.println(F("\nNo poses captured.")); return; }
  Serial.println(F("\n---------- WAYPOINTS ----------"));
  Serial.println(F("const Waypoint seq[] = {"));
  for (uint8_t i = 0; i < poseCount; i++) printPoseLine(i);
  Serial.println(F("};"));
  Serial.println(F("-------------------------------"));
}

void replayAll() {
  if (poseCount == 0) { Serial.println(F("Nothing to replay.")); return; }
  Serial.println(F("\nReplaying. Keys are NOT read during replay -- cut power if it goes wrong."));
  for (uint8_t i = 0; i < poseCount; i++) {
    Serial.print(F("  -> pose "));
    Serial.println(i);
    for (uint8_t j = 0; j < NUM_JOINTS; j++) sweepTo(j, pose[i][j]);
    delay(DEFAULT_DWELL);
  }
  Serial.println(F("Replay done."));
}

/* ---------- reporting ---------- */

void printStatus() {
  Serial.print(F("\n[ch "));
  Serial.print(selected);
  Serial.print(F(" "));
  Serial.print(JOINT_NAME[selected]);
  Serial.print(F("]  angle="));
  Serial.print(currentAngle[selected]);
  Serial.print(F("  ("));
  Serial.print(LIM_MIN[selected]);
  Serial.print(F(".."));
  Serial.print(LIM_MAX[selected]);
  Serial.print(F(")  step="));
  Serial.print(stepSize);
  Serial.print(F("  poses="));
  Serial.println(poseCount);

  Serial.print(F("  all: "));
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    Serial.print(currentAngle[j]);
    if (j < NUM_JOINTS - 1) Serial.print(F(", "));
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("\n=========== COMMANDS ==========="));
  Serial.println(F("  e     engage (arm at NEUTRAL by hand first)"));
  Serial.println(F("  !     RELAX ALL - SUPPORT THE ARM"));
  Serial.println(F("  0-5   select joint"));
  Serial.println(F("  + -   nudge selected joint"));
  Serial.println(F("  < >   step size 1-10 deg"));
  Serial.println(F("  o g   gripper open / grip"));
  Serial.println(F("  c     capture current pose as a waypoint"));
  Serial.println(F("  u     undo last capture"));
  Serial.println(F("  L     list all waypoints (paste-ready)"));
  Serial.println(F("  R     replay all captured waypoints"));
  Serial.println(F("  h     sweep selected joint to neutral"));
  Serial.println(F("  H     sweep ALL joints to neutral"));
  Serial.println(F("  s     show status"));
  Serial.println(F("  ?     this help"));
  Serial.println(F("================================"));
}

/* ---------- setup / loop ---------- */

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(50);
  delay(10);

  relaxAll();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) currentAngle[i] = NEUTRAL[i];

  Serial.println(F("\n\n=== POSE TEACHER ==="));
  Serial.println(F("Servos are NOT driven yet."));
  Serial.println(F("Place the arm at NEUTRAL by hand, then type 'e'."));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  if (c == '!') { relaxAll(); return; }
  if (c == '?') { printHelp(); return; }
  if (c == 'L') { listPoses(); return; }
  if (c == 'e') {
    if (engaged) Serial.println(F("Already engaged."));
    else         engage();
    return;
  }

  if (!engaged) {
    Serial.println(F("Not engaged. Arm to NEUTRAL by hand, then 'e'."));
    return;
  }

  switch (c) {
    case '0': case '1': case '2': case '3': case '4': case '5':
      selected = c - '0';
      printStatus();
      break;

    case '+': case '=':
      driveSafe(selected, (int16_t)currentAngle[selected] + stepSize);
      printStatus();
      break;

    case '-': case '_':
      driveSafe(selected, (int16_t)currentAngle[selected] - stepSize);
      printStatus();
      break;

    case '>': if (stepSize < 10) stepSize++; printStatus(); break;
    case '<': if (stepSize > 1)  stepSize--; printStatus(); break;

    case 'o': sweepTo(GRIPPER_CH, GRIPPER_OPEN);   printStatus(); break;
    case 'g': sweepTo(GRIPPER_CH, GRIPPER_CLOSED); printStatus(); break;

    case 'c': capturePose(); break;
    case 'u': undoPose();    break;
    case 'R': replayAll();   break;

    case 'h': sweepTo(selected, NEUTRAL[selected]); printStatus(); break;
    case 'H':
      for (uint8_t i = 0; i < NUM_JOINTS; i++) sweepTo(i, NEUTRAL[i]);
      Serial.println(F("All joints at neutral."));
      break;

    case 's': printStatus(); break;

    default:
      Serial.println(F("Unknown command. '?' for help."));
      break;
  }
}
