#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;


// ============================================================
// PCA9685 Servo Driver
// ============================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// ============================================================
// General Constants
// ============================================================

// Number of joints in the robot
const int NUM_JOINTS = 5;

// Servo PWM frequency
const int SERVO_FREQ = 50;


// ============================================================
// DHLink Class
//
// Represents one row of the Denavit-Hartenberg table.
//
// Each link has:
//
// r     = distance along x_i
// alpha = angle between z_(i-1) and z_i
// d     = distance along z_(i-1)
// theta = joint rotation
//
// ============================================================

class DHLink
{
private:

    // DH parameters
    float r;
    float alpha;
    float d;
    float theta;

public:

    // --------------------------------------------------------
    // Constructor
    //
    // rValue     = link length
    // alphaValue = twist angle in radians
    // dValue     = link offset
    //
    // theta starts at zero.
    // --------------------------------------------------------

    DHLink(float rValue,
           float alphaValue,
           float dValue)
    {
        r = rValue;
        alpha = alphaValue;
        d = dValue;
        theta = 0.0f;
    }


    // --------------------------------------------------------
    // Set joint angle using degrees
    // --------------------------------------------------------

    void setThetaDegrees(float thetaDegrees)
    {
        theta = thetaDegrees * PI / 180.0f;
    }


    // --------------------------------------------------------
    // Set joint angle directly in radians
    // --------------------------------------------------------

    void setThetaRadians(float thetaRadians)
    {
        theta = thetaRadians;
    }


    // --------------------------------------------------------
    // Return current theta value in radians
    // --------------------------------------------------------

    float getTheta() const
    {
        return theta;
    }


    // --------------------------------------------------------
    // Generate the DH homogeneous transformation matrix
    //
    // Standard DH convention:
    //
    //      [ cosθ  -sinθcosα   sinθsinα   r cosθ ]
    // A =  [ sinθ   cosθcosα  -cosθsinα   r sinθ ]
    //      [  0        sinα       cosα       d   ]
    //      [  0         0          0         1   ]
    //
    // --------------------------------------------------------

    Matrix<4, 4> getTransformationMatrix() const
    {
        float ct = cos(theta);
        float st = sin(theta);

        float ca = cos(alpha);
        float sa = sin(alpha);


        Matrix<4, 4> A = {

            ct,     -st * ca,      st * sa,      r * ct,

            st,      ct * ca,     -ct * sa,      r * st,

            0.0f,    sa,           ca,           d,

            0.0f,    0.0f,         0.0f,         1.0f
        };


        return A;
    }
};


// ============================================================
// RobotArm Class
//
// Represents the complete 5-DOF robot.
// It contains all five DH links and performs forward kinematics.
// ============================================================

class RobotArm
{
private:

    // --------------------------------------------------------
    // DH Table
    //
    // Link    r       alpha       d
    //
    // 1       0       pi/2       11.5
    // 2      10.5      0          0
    // 3       4.5      0          0
    // 4       6.5      0          0
    // 5       0       pi/2        2.5
    //
    // All distances are in centimeters.
    // --------------------------------------------------------

    DHLink link1;
    DHLink link2;
    DHLink link3;
    DHLink link4;
    DHLink link5;


public:

    // --------------------------------------------------------
    // Constructor
    //
    // Initialize each DH link using the robot dimensions.
    // --------------------------------------------------------

    RobotArm()
        : link1(0.0f,  PI / 2.0f, 11.5f),
          link2(10.5f, 0.0f,       0.0f),
          link3(4.5f,  0.0f,       0.0f),
          link4(6.5f,  0.0f,       0.0f),
          link5(0.0f,  PI / 2.0f,  2.5f)
    {
    }


    // --------------------------------------------------------
    // Set all five joint angles
    //
    // Inputs are in degrees.
    // --------------------------------------------------------

    void setJointAngles(float theta1,
                        float theta2,
                        float theta3,
                        float theta4,
                        float theta5)
    {
        link1.setThetaDegrees(theta1);
        link2.setThetaDegrees(theta2);
        link3.setThetaDegrees(theta3);
        link4.setThetaDegrees(theta4);
        link5.setThetaDegrees(theta5);
    }


    // --------------------------------------------------------
    // Calculate forward kinematics
    //
    // T05 = T01 * T12 * T23 * T34 * T45
    //
    // The resulting matrix describes the position and
    // orientation of frame 5 relative to frame 0.
    // --------------------------------------------------------

    Matrix<4, 4> forwardKinematics() const
    {
        // Get each individual transformation matrix

        Matrix<4, 4> T01 =
            link1.getTransformationMatrix();

        Matrix<4, 4> T12 =
            link2.getTransformationMatrix();

        Matrix<4, 4> T23 =
            link3.getTransformationMatrix();

        Matrix<4, 4> T34 =
            link4.getTransformationMatrix();

        Matrix<4, 4> T45 =
            link5.getTransformationMatrix();


        // Multiply all transformations

        Matrix<4, 4> T05 =
            T01 * T12 * T23 * T34 * T45;


        return T05;
    }


    // --------------------------------------------------------
    // Print the complete transformation matrix
    // --------------------------------------------------------

    void printTransformationMatrix() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();


        Serial.println("T05 = ");

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Serial.print(T05(i, j), 4);
                Serial.print("\t");
            }

            Serial.println();
        }

        Serial.println();
    }


    // --------------------------------------------------------
    // Print the end-effector position
    //
    // The transformation matrix has the form:
    //
    //      [ R11 R12 R13  x ]
    // T =  [ R21 R22 R23  y ]
    //      [ R31 R32 R33  z ]
    //      [  0   0   0   1 ]
    //
    // Therefore:
    //
    // x = T05(0,3)
    // y = T05(1,3)
    // z = T05(2,3)
    //
    // --------------------------------------------------------

    void printEndEffectorPosition() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();


        float x = T05(0, 3);
        float y = T05(1, 3);
        float z = T05(2, 3);


        Serial.println("End-Effector Position:");

        Serial.print("X = ");
        Serial.print(x, 4);
        Serial.println(" cm");

        Serial.print("Y = ");
        Serial.print(y, 4);
        Serial.println(" cm");

        Serial.print("Z = ");
        Serial.print(z, 4);
        Serial.println(" cm");

        Serial.println();
    }


    // --------------------------------------------------------
    // Return X position
    // --------------------------------------------------------

    float getX() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();

        return T05(0, 3);
    }


    // --------------------------------------------------------
    // Return Y position
    // --------------------------------------------------------

    float getY() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();

        return T05(1, 3);
    }


    // --------------------------------------------------------
    // Return Z position
    // --------------------------------------------------------

    float getZ() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();

        return T05(2, 3);
    }
};


// ============================================================
// Create Robot Object
// ============================================================

RobotArm robot;


// ============================================================
// setup()
//
// Runs once when Arduino starts.
// ============================================================

void setup()
{
    // Start serial communication

    Serial.begin(115200);


    // --------------------------------------------------------
    // Start I2C communication
    // --------------------------------------------------------
    Wire.begin();
    // --------------------------------------------------------
    // Initialize PCA9685 servo driver
    // --------------------------------------------------------

    pwm.begin();

    pwm.setOscillatorFrequency(27000000);

    pwm.setPWMFreq(SERVO_FREQ);


    delay(1000);


    Serial.println();
    Serial.println("5-DOF Robot Forward Kinematics");
    Serial.println();


    // --------------------------------------------------------
    // Example joint configuration
    //
    // Angles are given in degrees.
    // --------------------------------------------------------

    robot.setJointAngles(
        0.0f,      // theta1
        0.0f,      // theta2
        0.0f,      // theta3
        0.0f,      // theta4
        0.0f       // theta5
    );

    // Print results
    robot.printTransformationMatrix();
    robot.printEndEffectorPosition();
}


// ============================================================
// loop()
//
// Runs continuously after setup().
// ============================================================

void loop()
{
    // Nothing is required here yet.
}