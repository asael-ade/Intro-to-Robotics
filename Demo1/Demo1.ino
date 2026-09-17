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

// Standard hobby servo frequency
const int SERVO_FREQ = 50;


// ============================================================
// ServoJoint Class
//
// Handles:
// - PWM channel
// - physical joint limits
// - servo direction
// - servo offset
// - PWM calibration
//
// ============================================================

class ServoJoint
{
private:

    // PCA9685 channel
    int channel;

    // Physical joint limits in degrees
    float minJointAngle;
    float maxJointAngle;

    // Servo angle corresponding to joint angle = 0 deg
    float servoOffset;

    // true if servo motion is opposite to joint motion
    bool reversed;

    // Calibrated PCA9685 pulse limits
    int minPulse;
    int maxPulse;

    // Current joint angle
    float currentJointAngle;


public:

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------

    ServoJoint(int pwmChannel,
               float minAngle,
               float maxAngle,
               float offset,
               bool reverseDirection,
               int minimumPulse,
               int maximumPulse)
    {
        channel = pwmChannel;

        minJointAngle = minAngle;
        maxJointAngle = maxAngle;

        servoOffset = offset;

        reversed = reverseDirection;

        minPulse = minimumPulse;
        maxPulse = maximumPulse;

        currentJointAngle = 0.0f;
    }


    // --------------------------------------------------------
    // Convert joint angle to servo angle
    //
    // Normal:
    // servoAngle = offset + jointAngle
    //
    // Reversed:
    // servoAngle = offset - jointAngle
    // --------------------------------------------------------

    float jointToServoAngle(float jointAngle) const
    {
        if (reversed)
        {
            return servoOffset - jointAngle;
        }
        else
        {
            return servoOffset + jointAngle;
        }
    }


    // --------------------------------------------------------
    // Convert servo angle to PCA9685 pulse
    //
    // Assumes servo range of approximately 0 to 180 degrees.
    //
    // minPulse and maxPulse must be calibrated.
    // --------------------------------------------------------

    int servoAngleToPulse(float servoAngle) const
    {
        // Constrain servo angle
        servoAngle = constrain(servoAngle, 0.0f, 180.0f);

        // Convert servo angle to PWM count
        int pulse = map(
            (int)servoAngle,
            0,
            180,
            minPulse,
            maxPulse
        );

        return pulse;
    }


    // --------------------------------------------------------
    // Move joint
    //
    // Requested angle is first constrained to the
    // physical limits of the joint.
    // --------------------------------------------------------

    void setAngle(float jointAngle)
    {
        // Enforce physical joint limits
        jointAngle = constrain(
            jointAngle,
            minJointAngle,
            maxJointAngle
        );

        currentJointAngle = jointAngle;


        // Convert joint angle to actual servo angle
        float servoAngle =
            jointToServoAngle(jointAngle);


        // Convert servo angle to PWM pulse
        int pulse =
            servoAngleToPulse(servoAngle);


        // Send command to PCA9685
        pwm.setPWM(
            channel,
            0,
            pulse
        );
    }


    // --------------------------------------------------------
    // Return current joint angle
    // --------------------------------------------------------

    float getAngle() const
    {
        return currentJointAngle;
    }


    // --------------------------------------------------------
    // Return minimum physical limit
    // --------------------------------------------------------

    float getMinAngle() const
    {
        return minJointAngle;
    }


    // --------------------------------------------------------
    // Return maximum physical limit
    // --------------------------------------------------------

    float getMaxAngle() const
    {
        return maxJointAngle;
    }
};


// ============================================================
// DHLink Class
//
// Represents one row of the DH table.
// ============================================================

class DHLink
{
private:

    float r;
    float alpha;
    float d;

    // Stored internally in radians
    float theta;


public:

    // --------------------------------------------------------
    // Constructor
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
    // Set theta using degrees
    // --------------------------------------------------------

    void setThetaDegrees(float thetaDegrees)
    {
        theta =
            thetaDegrees * PI / 180.0f;
    }


    // --------------------------------------------------------
    // Build DH transformation matrix
    // --------------------------------------------------------

    Matrix<4, 4> getTransformationMatrix() const
    {
        float ct = cos(theta);
        float st = sin(theta);

        float ca = cos(alpha);
        float sa = sin(alpha);


        Matrix<4, 4> A = {

            ct,      -st * ca,      st * sa,      r * ct,

            st,       ct * ca,     -ct * sa,      r * st,

            0.0f,     sa,           ca,           d,

            0.0f,     0.0f,         0.0f,         1.0f
        };


        return A;
    }
};


// ============================================================
// RobotArm Class
//
// Combines:
// - Servo control
// - physical joint limits
// - DH kinematics
//
// ============================================================

class RobotArm
{
private:

    // --------------------------------------------------------
    // Servo joints
    //
    // Parameters:
    //
    // channel
    // min joint angle
    // max joint angle
    // servo offset
    // reversed?
    // min PWM pulse
    // max PWM pulse
    //
    // IMPORTANT:
    // These are example values.
    // Calibrate for your actual robot.
    // --------------------------------------------------------

    ServoJoint joint1;
    ServoJoint joint2;
    ServoJoint joint3;
    ServoJoint joint4;
    ServoJoint joint5;


    // --------------------------------------------------------
    // DH links
    // --------------------------------------------------------

    DHLink link1;
    DHLink link2;
    DHLink link3;
    DHLink link4;
    DHLink link5;


public:

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------

    RobotArm()

        // Servo configuration
        : joint1(
              0,          // PCA9685 channel
              -90.0f,     // minimum joint angle
              90.0f,      // maximum joint angle
              90.0f,      // servo offset
              false,      // reversed?
              120,        // minimum pulse
              500         // maximum pulse
          ),

          joint2(
              1,
              0.0f,
              135.0f,
              20.0f,
              false,
              120,
              500
          ),

          joint3(
              2,
              -100.0f,
              100.0f,
              90.0f,
              true,
              120,
              500
          ),

          joint4(
              3,
              -90.0f,
              90.0f,
              90.0f,
              false,
              120,
              500
          ),

          joint5(
              4,
              -90.0f,
              90.0f,
              90.0f,
              false,
              120,
              500
          ),


          // DH table
          link1(
              0.0f,
              PI / 2.0f,
              11.5f
          ),

          link2(
              10.5f,
              0.0f,
              0.0f
          ),

          link3(
              4.5f,
              0.0f,
              0.0f
          ),

          link4(
              6.5f,
              0.0f,
              0.0f
          ),

          link5(
              0.0f,
              PI / 2.0f,
              2.5f
          )

    {
    }


    // --------------------------------------------------------
    // Set all joint angles
    //
    // This function:
    //
    // 1. Sends safe commands to the servos
    // 2. Retrieves the actual constrained angles
    // 3. Updates the DH model
    //
    // --------------------------------------------------------

    void setJointAngles(float theta1,
                        float theta2,
                        float theta3,
                        float theta4,
                        float theta5)
    {
        // Move servos
        joint1.setAngle(theta1);
        joint2.setAngle(theta2);
        joint3.setAngle(theta3);
        joint4.setAngle(theta4);
        joint5.setAngle(theta5);


        // Update DH model using actual constrained angles
        link1.setThetaDegrees(joint1.getAngle());
        link2.setThetaDegrees(joint2.getAngle());
        link3.setThetaDegrees(joint3.getAngle());
        link4.setThetaDegrees(joint4.getAngle());
        link5.setThetaDegrees(joint5.getAngle());
    }


    // --------------------------------------------------------
    // Compute forward kinematics
    //
    // T05 = T01 * T12 * T23 * T34 * T45
    // --------------------------------------------------------

    Matrix<4, 4> forwardKinematics() const
    {
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


        Matrix<4, 4> T05 =
            T01 * T12 * T23 * T34 * T45;


        return T05;
    }


    // --------------------------------------------------------
    // Print current constrained joint angles
    // --------------------------------------------------------

    void printJointAngles() const
    {
        Serial.println("Joint Angles:");

        Serial.print("Theta 1 = ");
        Serial.println(joint1.getAngle(), 2);

        Serial.print("Theta 2 = ");
        Serial.println(joint2.getAngle(), 2);

        Serial.print("Theta 3 = ");
        Serial.println(joint3.getAngle(), 2);

        Serial.print("Theta 4 = ");
        Serial.println(joint4.getAngle(), 2);

        Serial.print("Theta 5 = ");
        Serial.println(joint5.getAngle(), 2);

        Serial.println();
    }


    // --------------------------------------------------------
    // Print physical joint limits
    // --------------------------------------------------------

    void printJointLimits() const
    {
        Serial.println("Joint Physical Limits:");

        Serial.print("Joint 1: ");
        Serial.print(joint1.getMinAngle());
        Serial.print(" to ");
        Serial.println(joint1.getMaxAngle());

        Serial.print("Joint 2: ");
        Serial.print(joint2.getMinAngle());
        Serial.print(" to ");
        Serial.println(joint2.getMaxAngle());

        Serial.print("Joint 3: ");
        Serial.print(joint3.getMinAngle());
        Serial.print(" to ");
        Serial.println(joint3.getMaxAngle());

        Serial.print("Joint 4: ");
        Serial.print(joint4.getMinAngle());
        Serial.print(" to ");
        Serial.println(joint4.getMaxAngle());

        Serial.print("Joint 5: ");
        Serial.print(joint5.getMinAngle());
        Serial.print(" to ");
        Serial.println(joint5.getMaxAngle());

        Serial.println();
    }


    // --------------------------------------------------------
    // Print transformation matrix
    // --------------------------------------------------------

    void printTransformationMatrix() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();


        Serial.println("T05 = ");

        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                Serial.print(
                    T05(row, col),
                    4
                );

                Serial.print("\t");
            }

            Serial.println();
        }

        Serial.println();
    }


    // --------------------------------------------------------
    // Print end-effector position
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
        Serial.print(x, 3);
        Serial.println(" cm");

        Serial.print("Y = ");
        Serial.print(y, 3);
        Serial.println(" cm");

        Serial.print("Z = ");
        Serial.print(z, 3);
        Serial.println(" cm");

        Serial.println();
    }
};


// ============================================================
// Create Robot Object
// ============================================================

RobotArm robot;


// ============================================================
// setup()
// ============================================================

void setup()
{
    // Start serial communication
    Serial.begin(115200);


    // Start I2C communication
    Wire.begin();


    // Initialize PCA9685
    pwm.begin();


    // PCA9685 oscillator frequency
    pwm.setOscillatorFrequency(27000000);


    // MG996R uses standard servo PWM
    pwm.setPWMFreq(SERVO_FREQ);


    // Allow hardware to initialize
    delay(1000);


    Serial.println();
    Serial.println("MG996R 5-DOF Robot Arm");
    Serial.println();


    // Print configured limits
    robot.printJointLimits();


    // --------------------------------------------------------
    // Initial robot pose
    //
    // Start with a conservative pose.
    // --------------------------------------------------------

    robot.setJointAngles(
        0.0f,     // theta1
        45.0f,    // theta2
        0.0f,     // theta3
        0.0f,     // theta4
        0.0f      // theta5
    );


    // Give the servos time to move
    delay(1500);


    // Print current joint angles
    robot.printJointAngles();


    // Print forward kinematics
    robot.printTransformationMatrix();

    robot.printEndEffectorPosition();
}


// ============================================================
// loop()
// ============================================================

void loop()
{
    // Robot motion can be added here later.
}