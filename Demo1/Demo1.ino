#include <Arduino.h>
#include <math.h>

// =====================================================
// Robot dimensions
// Change these values to match your robot
// =====================================================

float d1 = 10.0;

float r2 = 10.0;
float r3 = 5.0;
float r4 = 5.0;
float r5 = 2.0;

float d5 = 2.0;


// =====================================================
// Joint angles in degrees
// =====================================================

float theta1 = 0.0;
float theta2 = 0.0;
float theta3 = 0.0;
float theta4 = 0.0;
float theta5 = 0.0;


// =====================================================
// Convert degrees to radians
// =====================================================

float deg2rad(float angleDeg)
{
  return angleDeg * PI / 180.0;
}


// =====================================================
// Standard DH Transformation Matrix
//
// Parameters:
// r     = link length
// alpha = link twist
// d     = link offset
// theta = joint angle
//
// T =
// [ cosθ  -sinθcosα   sinθsinα   rcosθ ]
// [ sinθ   cosθcosα  -cosθsinα   rsinθ ]
// [  0        sinα       cosα       d   ]
// [  0          0          0         1   ]
// =====================================================

void DHTransform(float r,
                 float alpha,
                 float d,
                 float theta,
                 float T[4][4])
{
  float ct = cos(theta);
  float st = sin(theta);

  float ca = cos(alpha);
  float sa = sin(alpha);

  T[0][0] = ct;
  T[0][1] = -st * ca;
  T[0][2] = st * sa;
  T[0][3] = r * ct;

  T[1][0] = st;
  T[1][1] = ct * ca;
  T[1][2] = -ct * sa;
  T[1][3] = r * st;

  T[2][0] = 0;
  T[2][1] = sa;
  T[2][2] = ca;
  T[2][3] = d;

  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;
}


// =====================================================
// Multiply two 4x4 matrices
//
// C = A * B
// =====================================================

void matrixMultiply(float A[4][4],
                    float B[4][4],
                    float C[4][4])
{
  float temp[4][4];

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      temp[i][j] = 0;

      for (int k = 0; k < 4; k++)
      {
        temp[i][j] += A[i][k] * B[k][j];
      }
    }
  }

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      C[i][j] = temp[i][j];
    }
  }
}


// =====================================================
// Print 4x4 matrix
// =====================================================

void printMatrix(float M[4][4])
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Serial.print(M[i][j], 4);
      Serial.print("\t");
    }

    Serial.println();
  }

  Serial.println();
}


// =====================================================
// Forward Kinematics
// =====================================================

void forwardKinematics()
{
  // Individual transformation matrices

  float T01[4][4];
  float T12[4][4];
  float T23[4][4];
  float T34[4][4];
  float T45[4][4];

  // Combined matrices

  float T02[4][4];
  float T03[4][4];
  float T04[4][4];
  float T05[4][4];


  // Convert angles from degrees to radians

  float th1 = deg2rad(theta1);
  float th2 = deg2rad(theta2);
  float th3 = deg2rad(theta3);
  float th4 = deg2rad(theta4);
  float th5 = deg2rad(theta5);


  // ===================================================
  // DH TABLE
  //
  // Link | r   | alpha | d  | theta
  // ----------------------------------
  // 1    | 0   | pi/2  | d1 | theta1
  // 2    | r2  | 0     | 0  | theta2
  // 3    | r3  | 0     | 0  | theta3
  // 4    | r4  | 0     | 0  | theta4
  // 5    | r5  | pi/2  | d5 | theta5
  // ===================================================


  // Link 1
  DHTransform(
    0,
    PI / 2,
    d1,
    th1,
    T01
  );


  // Link 2
  DHTransform(
    r2,
    0,
    0,
    th2,
    T12
  );


  // Link 3
  DHTransform(
    r3,
    0,
    0,
    th3,
    T23
  );


  // Link 4
  DHTransform(
    r4,
    0,
    0,
    th4,
    T34
  );


  // Link 5
  DHTransform(
    r5,
    PI / 2,
    d5,
    th5,
    T45
  );


  // ===================================================
  // Calculate complete transformation
  //
  // T05 = T01 * T12 * T23 * T34 * T45
  // ===================================================

  matrixMultiply(T01, T12, T02);

  matrixMultiply(T02, T23, T03);

  matrixMultiply(T03, T34, T04);

  matrixMultiply(T04, T45, T05);


  // ===================================================
  // Print individual transformations
  // ===================================================

  Serial.println("T01:");
  printMatrix(T01);

  Serial.println("T12:");
  printMatrix(T12);

  Serial.println("T23:");
  printMatrix(T23);

  Serial.println("T34:");
  printMatrix(T34);

  Serial.println("T45:");
  printMatrix(T45);


  // ===================================================
  // Print final transformation
  // ===================================================

  Serial.println("T05:");
  printMatrix(T05);


  // ===================================================
  // Extract end-effector position
  // ===================================================

  float x = T05[0][3];
  float y = T05[1][3];
  float z = T05[2][3];


  Serial.println("End Effector Position:");

  Serial.print("X = ");
  Serial.println(x, 4);

  Serial.print("Y = ");
  Serial.println(y, 4);

  Serial.print("Z = ");
  Serial.println(z, 4);

  Serial.println();
}


// =====================================================
// Arduino setup
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("5-DOF Robot Forward Kinematics");
  Serial.println("==============================");

  forwardKinematics();
}


// =====================================================
// Arduino loop
// =====================================================

void loop()
{
}