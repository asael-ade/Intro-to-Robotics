#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// =====================================================
// PCA9685
// =====================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels on PCA9685
const int servoChannel[5] = {
  0, 1, 2, 3, 4
};

// Servo pulse limits
// These may need calibration for your servos
const int SERVOMIN = 110;
const int SERVOMAX = 510;


// =====================================================
// Servo zero offsets
//
// Physical servo position corresponding to
// theta_i = 0 in the DH model
// =====================================================

float servoZero[5] = {
  90,
  90,
  90,
  90,
  90
};


// =====================================================
// Servo direction
//
// Use 1 if positive DH angle means increasing servo angle
// Use -1 if positive DH angle means decreasing servo angle
// =====================================================

int servoDirection[5] = {
  1,
  1,
  1,
  1,
  1
};


// =====================================================
// Joint limits in DH coordinates
// Adjust for your robot
// =====================================================

float jointMin[5] = {
  -90,
  -90,
  -90,
  -90,
  -90
};

float jointMax[5] = {
   90,
   90,
   90,
   90,
   90
};


// =====================================================
// Robot dimensions
// =====================================================

float d1 = 10.0;

float r2 = 10.0;
float r3 = 5.0;
float r4 = 5.0;
float r5 = 2.0;

float d5 = 2.0;


// =====================================================
// Degrees to radians
// =====================================================

float deg2rad(float angle)
{
  return angle * PI / 180.0;
}


// =====================================================
// Convert servo angle to PCA9685 pulse
// =====================================================

int angleToPulse(float angle)
{
  angle = constrain(angle, 0, 180);

  return map(
    (int)angle,
    0,
    180,
    SERVOMIN,
    SERVOMAX
  );
}


// =====================================================
// Move one servo
// =====================================================

void setServoAngle(int joint, float dhAngle)
{
  // Apply DH joint limits
  dhAngle = constrain(
    dhAngle,
    jointMin[joint],
    jointMax[joint]
  );

  // Convert DH angle to physical servo angle
  float servoAngle =
    servoZero[joint]
    +
    servoDirection[joint] * dhAngle;

  servoAngle = constrain(
    servoAngle,
    0,
    180
  );

  int pulse = angleToPulse(servoAngle);

  pwm.setPWM(
    servoChannel[joint],
    0,
    pulse
  );
}


// =====================================================
// Standard DH Transformation
// =====================================================

Matrix<4,4> DH(
  float r,
  float alpha,
  float d,
  float theta
)
{
  float ct = cos(theta);
  float st = sin(theta);

  float ca = cos(alpha);
  float sa = sin(alpha);

  Matrix<4,4> T = {

    ct,  -st * ca,   st * sa,   r * ct,

    st,   ct * ca,  -ct * sa,   r * st,

    0,         sa,        ca,        d,

    0,          0,         0,        1
  };

  return T;
}


// =====================================================
// Forward Kinematics
// =====================================================

Matrix<4,4> forwardKinematics(
  float t1,
  float t2,
  float t3,
  float t4,
  float t5
)
{
  t1 = deg2rad(t1);
  t2 = deg2rad(t2);
  t3 = deg2rad(t3);
  t4 = deg2rad(t4);
  t5 = deg2rad(t5);


  Matrix<4,4> T01 =
    DH(0, PI/2, d1, t1);

  Matrix<4,4> T12 =
    DH(r2, 0, 0, t2);

  Matrix<4,4> T23 =
    DH(r3, 0, 0, t3);

  Matrix<4,4> T34 =
    DH(r4, 0, 0, t4);

  Matrix<4,4> T45 =
    DH(r5, PI/2, d5, t5);


  return
    T01 *
    T12 *
    T23 *
    T34 *
    T45;
}


// =====================================================
// Move complete robot
// =====================================================

void moveRobot(
  float t1,
  float t2,
  float t3,
  float t4,
  float t5
)
{
  float theta[5] = {
    t1, t2, t3, t4, t5
  };


  // Move each servo
  for (int i = 0; i < 5; i++)
  {
    setServoAngle(i, theta[i]);
  }


  // Calculate forward kinematics
  Matrix<4,4> T05 =
    forwardKinematics(
      t1,
      t2,
      t3,
      t4,
      t5
    );


  float x = T05(0,3);
  float y = T05(1,3);
  float z = T05(2,3);


  Serial.println();
  Serial.println("Joint Angles");

  for (int i = 0; i < 5; i++)
  {
    Serial.print("Theta ");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.println(theta[i]);
  }


  Serial.println();
  Serial.println("End Effector Position");

  Serial.print("X = ");
  Serial.println(x, 3);

  Serial.print("Y = ");
  Serial.println(y, 3);

  Serial.print("Z = ");
  Serial.println(z, 3);

  Serial.println();
}


// =====================================================
// Home position
// =====================================================

void homeRobot()
{
  moveRobot(
    0,
    0,
    0,
    0,
    0
  );
}


// =====================================================
// Setup
// =====================================================

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  pwm.begin();

  // Standard analog servo frequency
  pwm.setPWMFreq(50);

  delay(10);


  Serial.println();
  Serial.println("===========================");
  Serial.println("5-DOF Robot with PCA9685");
  Serial.println("===========================");

  Serial.println();
  Serial.println("Enter:");

  Serial.println(
    "theta1 theta2 theta3 theta4 theta5"
  );

  Serial.println();

  Serial.println("Example:");

  Serial.println(
    "30 45 -20 10 30"
  );

  Serial.println();

  Serial.println(
    "Type HOME for zero configuration."
  );


  homeRobot();
}


// =====================================================
// Loop
// =====================================================

void loop()
{
  if (Serial.available())
  {
    String command =
      Serial.readStringUntil('\n');

    command.trim();


    if (command.equalsIgnoreCase("HOME"))
    {
      homeRobot();
      return;
    }


    float t1;
    float t2;
    float t3;
    float t4;
    float t5;


    int values = sscanf(
      command.c_str(),
      "%f %f %f %f %f",
      &t1,
      &t2,
      &t3,
      &t4,
      &t5
    );


    if (values == 5)
    {
      moveRobot(
        t1,
        t2,
        t3,
        t4,
        t5
      );
    }

    else
    {
      Serial.println(
        "Invalid input."
      );

      Serial.println(
        "Example: 30 45 -20 10 30"
      );
    }
  }
}