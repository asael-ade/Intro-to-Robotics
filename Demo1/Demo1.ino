#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// =====================================================
// Servos
// =====================================================

Servo servo[5];

const int servoPins[5] = {
  3, 5, 6, 9, 10
};


// =====================================================
// Servo zero positions
//
// This is the physical servo angle corresponding
// to theta_i = 0 degrees in your DH model.
//
// Adjust these after calibrating your robot.
// =====================================================

float servoZero[5] = {
  90,
  90,
  90,
  90,
  90
};


// =====================================================
// Robot dimensions
//
// Replace these with your actual dimensions.
// Use the same units for every length.
// =====================================================

float d1 = 10.0;

float r2 = 10.0;
float r3 = 5.0;
float r4 = 5.0;
float r5 = 2.0;

float d5 = 2.0;


// =====================================================
// Convert degrees to radians
// =====================================================

float deg2rad(float degrees)
{
  return degrees * PI / 180.0;
}


// =====================================================
// DH Transformation
//
// Standard DH:
//
// A_i = Rot(z,theta)
//       Trans(z,d)
//       Trans(x,r)
//       Rot(x,alpha)
// =====================================================

Matrix<4,4> DH(float r,
               float alpha,
               float d,
               float theta)
{
  float ct = cos(theta);
  float st = sin(theta);

  float ca = cos(alpha);
  float sa = sin(alpha);

  Matrix<4,4> T = {
     ct,   -st * ca,    st * sa,    r * ct,
     st,    ct * ca,   -ct * sa,    r * st,
      0,         sa,         ca,         d,
      0,          0,          0,         1
  };

  return T;
}


// =====================================================
// Forward Kinematics
// =====================================================

Matrix<4,4> forwardKinematics(float t1,
                              float t2,
                              float t3,
                              float t4,
                              float t5)
{
  // Convert joint angles to radians

  t1 = deg2rad(t1);
  t2 = deg2rad(t2);
  t3 = deg2rad(t3);
  t4 = deg2rad(t4);
  t5 = deg2rad(t5);


  // DH table from your drawing
  //
  // Link    r       alpha      d       theta
  //
  // 1       0       pi/2       d1      theta1
  // 2       r2      0          0       theta2
  // 3       r3      0          0       theta3
  // 4       r4      0          0       theta4
  // 5       r5      pi/2       d5      theta5


  Matrix<4,4> T01 = DH(
    0,
    PI / 2,
    d1,
    t1
  );

  Matrix<4,4> T12 = DH(
    r2,
    0,
    0,
    t2
  );

  Matrix<4,4> T23 = DH(
    r3,
    0,
    0,
    t3
  );

  Matrix<4,4> T34 = DH(
    r4,
    0,
    0,
    t4
  );

  Matrix<4,4> T45 = DH(
    r5,
    PI / 2,
    d5,
    t5
  );


  // Complete transformation

  Matrix<4,4> T05 =
      T01 *
      T12 *
      T23 *
      T34 *
      T45;

  return T05;
}


// =====================================================
// Move Robot
//
// Input angles are DH joint angles.
// =====================================================

void moveRobot(float t1,
               float t2,
               float t3,
               float t4,
               float t5)
{
  float theta[5] = {
    t1,
    t2,
    t3,
    t4,
    t5
  };


  for (int i = 0; i < 5; i++)
  {
    // Convert DH angle to physical servo position

    float servoAngle =
      servoZero[i] + theta[i];


    // Servo can only accept 0-180 degrees

    servoAngle = constrain(
      servoAngle,
      0,
      180
    );


    servo[i].write(servoAngle);
  }


  // Calculate FK

  Matrix<4,4> T05 =
    forwardKinematics(
      t1,
      t2,
      t3,
      t4,
      t5
    );


  // Position of end effector

  float x = T05(0,3);
  float y = T05(1,3);
  float z = T05(2,3);


  Serial.println();
  Serial.println("Robot moved to:");

  Serial.print("theta1 = ");
  Serial.println(t1);

  Serial.print("theta2 = ");
  Serial.println(t2);

  Serial.print("theta3 = ");
  Serial.println(t3);

  Serial.print("theta4 = ");
  Serial.println(t4);

  Serial.print("theta5 = ");
  Serial.println(t5);


  Serial.println();

  Serial.println("End Effector Position:");

  Serial.print("X = ");
  Serial.println(x, 3);

  Serial.print("Y = ");
  Serial.println(y, 3);

  Serial.print("Z = ");
  Serial.println(z, 3);

  Serial.println();
}


// =====================================================
// HOME
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


  // Attach all five servos

  for (int i = 0; i < 5; i++)
  {
    servo[i].attach(
      servoPins[i]
    );

    servo[i].write(
      servoZero[i]
    );
  }


  delay(500);


  Serial.println();
  Serial.println("===============================");
  Serial.println("5-DOF Robot Controller");
  Serial.println("===============================");

  Serial.println();

  Serial.println(
    "Enter DH joint angles:"
  );

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


    // HOME command

    if (command.equalsIgnoreCase("HOME"))
    {
      homeRobot();

      return;
    }


    // Read angles

    float t1;
    float t2;
    float t3;
    float t4;
    float t5;


    int values =
      sscanf(
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
        "Invalid command."
      );

      Serial.println(
        "Example: 30 45 -20 10 30"
      );
    }
  }
}