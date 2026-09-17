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

// Standard servo frequency for MG996R
const int SERVO_FREQ = 50;


// ============================================================
// ServoJoint Class
//
// Controls one physical revolute joint.
//
// Stores:
// - PCA9685 channel
// - joint limits
// - servo offset
// - servo direction
// - PWM limits
// ============================================================

class ServoJoint
{
private:

    int channel;

    float minJointAngle;
    float maxJointAngle;

    float servoOffset;

    bool reversed;

    int minPulse;
    int maxPulse;

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
    // Convert robot joint angle to servo angle
    // --------------------------------------------------------

    float jointToServoAngle(float jointAngle) const
    {
        if (reversed)
        {
            return servoOffset - jointAngle;
        }

        return servoOffset + jointAngle;
    }


    // --------------------------------------------------------
    // Convert servo angle to PCA9685 pulse
    // --------------------------------------------------------

    int servoAngleToPulse(float servoAngle) const
    {
        // Keep servo command inside 0 to 180 degrees
        servoAngle = constrain(
            servoAngle,
            0.0f,
            180.0f
        );


        // Convert 0-180 degrees into PCA9685 pulse count
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
    // Move physical joint
    // --------------------------------------------------------

    void setAngle(float jointAngle)
    {
        // Apply physical joint limits
        jointAngle = constrain(
            jointAngle,
            minJointAngle,
            maxJointAngle
        );


        currentJointAngle = jointAngle;


        // Convert robot angle into actual servo angle
        float servoAngle =
            jointToServoAngle(jointAngle);


        // Convert servo angle into PWM value
        int pulse =
            servoAngleToPulse(servoAngle);


        // Send PWM command
        pwm.setPWM(
            channel,
            0,
            pulse
        );
    }


    // --------------------------------------------------------
    // Return current angle
    // --------------------------------------------------------

    float getAngle() const
    {
        return currentJointAngle;
    }


    // --------------------------------------------------------
    // Return minimum joint limit
    // --------------------------------------------------------

    float getMinAngle() const
    {
        return minJointAngle;
    }


    // --------------------------------------------------------
    // Return maximum joint limit
    // --------------------------------------------------------

    float getMaxAngle() const
    {
        return maxJointAngle;
    }
};


// ============================================================
// Claw Class
//
// Controls the gripper independently from the robot DH chain.
//
// The claw is not treated as an additional kinematic joint.
// It simply opens and closes using another servo.
// ============================================================

class Claw
{
private:

    // PCA9685 channel used by claw servo
    int channel;

    // Servo angle corresponding to open position
    float openAngle;

    // Servo angle corresponding to closed position
    float closedAngle;

    // PWM calibration
    int minPulse;
    int maxPulse;

    // Keep track of claw state
    bool clawClosed;


    // --------------------------------------------------------
    // Convert servo angle to PWM pulse
    // --------------------------------------------------------

    int angleToPulse(float angle) const
    {
        angle = constrain(
            angle,
            0.0f,
            180.0f
        );


        return map(
            (int)angle,
            0,
            180,
            minPulse,
            maxPulse
        );
    }


public:

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------

    Claw(int pwmChannel,
         float openPosition,
         float closedPosition,
         int minimumPulse,
         int maximumPulse)
    {
        channel = pwmChannel;

        openAngle = openPosition;
        closedAngle = closedPosition;

        minPulse = minimumPulse;
        maxPulse = maximumPulse;

        clawClosed = false;
    }


    // --------------------------------------------------------
    // Open claw
    // --------------------------------------------------------

    void open()
    {
        int pulse =
            angleToPulse(openAngle);


        pwm.setPWM(
            channel,
            0,
            pulse
        );


        clawClosed = false;
    }


    // --------------------------------------------------------
    // Close claw
    // --------------------------------------------------------

    void close()
    {
        int pulse =
            angleToPulse(closedAngle);


        pwm.setPWM(
            channel,
            0,
            pulse
        );


        clawClosed = true;
    }


    // --------------------------------------------------------
    // Return claw state
    // --------------------------------------------------------

    bool isClosed() const
    {
        return clawClosed;
    }
};


// ============================================================
// DHLink Class
//
// Represents one DH transformation.
// ============================================================

class DHLink
{
private:

    float r;
    float alpha;
    float d;

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
    // Set theta in degrees
    // --------------------------------------------------------

    void setThetaDegrees(float thetaDegrees)
    {
        theta =
            thetaDegrees * PI / 180.0f;
    }


    // --------------------------------------------------------
    // Build standard DH transformation matrix
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
// Controls:
// - 5 arm joints
// - forward kinematics
// - claw
// ============================================================

class RobotArm
{
private:

    // --------------------------------------------------------
    // Physical servo joints
    //
    // Parameters:
    //
    // channel
    // min angle
    // max angle
    // offset
    // reversed?
    // min pulse
    // max pulse
    //
    // These values should be calibrated.
    // --------------------------------------------------------

    ServoJoint joint1;
    ServoJoint joint2;
    ServoJoint joint3;
    ServoJoint joint4;
    ServoJoint joint5;


    // --------------------------------------------------------
    // Separate claw servo
    //
    // Channel 5
    //
    // Example:
    // 90 degrees = open
    // 20 degrees = closed
    //
    // Calibrate these values for your claw.
    // --------------------------------------------------------

    Claw claw;


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

        // -----------------------
        // Joint servo definitions
        // -----------------------

        : joint1(
              0,
              -90.0f,
              90.0f,
              90.0f,
              false,
              120,
              500
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


          // -----------------------
          // Claw servo definition
          // -----------------------

          claw(
              5,          // PCA9685 channel
              90.0f,      // open angle
              20.0f,      // closed angle
              120,        // minimum pulse
              500         // maximum pulse
          ),


          // -----------------------
          // DH table
          // -----------------------

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
    // Set arm joint angles
    // --------------------------------------------------------

    void setJointAngles(float theta1,
                        float theta2,
                        float theta3,
                        float theta4,
                        float theta5)
    {
        // Move physical servos
        joint1.setAngle(theta1);
        joint2.setAngle(theta2);
        joint3.setAngle(theta3);
        joint4.setAngle(theta4);
        joint5.setAngle(theta5);


        // Update DH model using constrained angles
        link1.setThetaDegrees(joint1.getAngle());
        link2.setThetaDegrees(joint2.getAngle());
        link3.setThetaDegrees(joint3.getAngle());
        link4.setThetaDegrees(joint4.getAngle());
        link5.setThetaDegrees(joint5.getAngle());
    }


    // --------------------------------------------------------
    // Open claw
    // --------------------------------------------------------

    void openClaw()
    {
        claw.open();

        Serial.println("Claw opened.");
    }


    // --------------------------------------------------------
    // Close claw
    // --------------------------------------------------------

    void closeClaw()
    {
        claw.close();

        Serial.println("Claw closed.");
    }


    // --------------------------------------------------------
    // Compute forward kinematics
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
    // Print end-effector position
    // --------------------------------------------------------

    void printEndEffectorPosition() const
    {
        Matrix<4, 4> T05 =
            forwardKinematics();


        Serial.println("End-Effector Position:");

        Serial.print("X = ");
        Serial.print(T05(0, 3), 3);
        Serial.println(" cm");

        Serial.print("Y = ");
        Serial.print(T05(1, 3), 3);
        Serial.println(" cm");

        Serial.print("Z = ");
        Serial.print(T05(2, 3), 3);
        Serial.println(" cm");

        Serial.println();
    }


    // --------------------------------------------------------
    // Print joint angles
    // --------------------------------------------------------

    void printJointAngles() const
    {
        Serial.println("Joint Angles:");

        Serial.print("Theta 1 = ");
        Serial.println(joint1.getAngle());

        Serial.print("Theta 2 = ");
        Serial.println(joint2.getAngle());

        Serial.print("Theta 3 = ");
        Serial.println(joint3.getAngle());

        Serial.print("Theta 4 = ");
        Serial.println(joint4.getAngle());

        Serial.print("Theta 5 = ");
        Serial.println(joint5.getAngle());

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
    // Start serial monitor
    Serial.begin(115200);


    // Start I2C
    Wire.begin();


    // Initialize PCA9685
    pwm.begin();

    pwm.setOscillatorFrequency(27000000);

    pwm.setPWMFreq(SERVO_FREQ);


    delay(1000);


    Serial.println();
    Serial.println("5-DOF Robot Arm with Claw");
    Serial.println();


    // --------------------------------------------------------
    // Open claw initially
    // --------------------------------------------------------

    robot.openClaw();


    // --------------------------------------------------------
    // Move robot to initial position
    // --------------------------------------------------------

    robot.setJointAngles(
        0.0f,
        45.0f,
        0.0f,
        0.0f,
        0.0f
    );


    delay(1500);


    // Print kinematic information
    robot.printJointAngles();

    robot.printEndEffectorPosition();


    // --------------------------------------------------------
    // Example claw operation
    //
    // Wait, then close claw.
    // --------------------------------------------------------

    delay(2000);

    robot.closeClaw();
}


// ============================================================
// loop()
// ============================================================

void loop()
{
    // Future robot sequences can be added here.
}