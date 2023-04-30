//#include "Servo.hpp"
#include "Arm.hpp"
#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"
#include <cmath>

// GLOBALS
const float L1 = 94.0;          // 9.4 cm (from ground)
const float L2 = 97.0;          // 9.7 cm
const float L3 = 54.0;          // 5.4 cm
const int openM3 = 80;          // Servo angle for open claw
const int closeM3 = 180;        // Servo angle for closed claw


namespace Hardware {
namespace Arm {
    Arm::Arm(): Servo1{}, Servo2(), Servo3, Servo4{
        Hardware::Servos::Servo Servo1{Hardware::Processor::SERVO1_PWM};
        Servo1.enable();
        Hardware::Servos::Servo Servo2{Hardware::Processor::SERVO2_PWM};
        Servo2.enable();
        Hardware::Servos::Servo Servo3{Hardware::Processor::SERVO2_PWM};
        Servo3.enable();
        Hardware::Servos::Servo Servo4{Hardware::Processor::SERVO3_PWM};
        Servo4.enable();
    }

    void Arm::resetPosition(float currentPosition1, float currentPosition2, float currentPosition3, float currentPosition4){
        float endPosition1{90.0};
        float endPosition2{30.0};
        float endPosition3{100.0}; 
        float endPosition4{80.0};
        Servo1.sweep(currentPosition1, endPosition1, Servo1.swe);
        Servo2.sweep(currentPosition2, endPosition2, Servo::SweepConfiguration{});
        Servo3.sweep(currentPosition3, endPosition3, Servo::SweepConfiguration{});
        Servo4.sweep(currentPosition4, endPosition4, Servo::SweepConfiguration{});
    }

    void Arm::compute_ik(float x, float y, float z, float &theta1, float &theta2, float &theta3){
        
    }

    void Arm::dumpArmMotion(float currentPositionServo1, float currentPositionServo2, float currentPositionServo3){
        // Assumes that the arm is currently in a reset position
        int reverseS1 = 0;
        int reverseS2 = 180;

        // Flip arm over and open/close claw
        Servo1.sweep(currentPositionServo1, reverseS1, Servo1.sweepParams);
        Servo2.sweep(currentPositionServo2, reverseS2, Servo2.sweepParams);
        Servo3.sweep(currentPosition3, openM3, Servo3.sweepParams);
        Cy_SysLib_Delay(2000);
        Servo3.sweep(Servo3.read(), closeM3, Servo3.sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset Arm Position
        resetPosition(reverseS1, reverseS2, closeM3);
    }

    void Arm::moveArmForward(float theta1, float theta2, float theta3){
        int M0 = Servo_0.read();
        int M1 = Servo_1.read();
        int M2 = Servo_2.read();
        int M3 = Servo_3.read();
        // Assumes that the arm is currently in a reset position
        Servo1.sweep(M0, theta1, Servo1.sweepParams);
        Servo2.sweep(M1, theta2, Servo2.sweepParams);
        Servo3.sweep(M2, theta3, Servo3.sweepParams);
        Servo4.sweep(M3, openM3, Servo4.sweepParams);
        Cy_SysLib_Delay(1000);
        Servo4.sweep(openM3, closeM3, Servo4.sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset to starting position
        resetPosition(theta1, theta2, theta3, closeM3);
    }


}



} // namespace Arm
} // namespace Hardware

//Hardware::Servos::Servo Servo1{Hardware::Processor::SERVO1_PWM};          // shoulder
//Hardware::Servos::Servo Servo2{Hardware::Processor::SERVO2_PWM};          // elbow
//Hardware::Servos::Servo Servo3{Hardware::Processor::SERVO2_PWM};          // wrist
//Hardware::Servos::Servo Servo4{Hardware::Processor::SERVO3_PWM};          // claw


///////////////////////////////////////////////////
// Initial Function for IK
///////////////////////////////////////////////////
void inverseKinematics(double x,double y,double z,double L1,double L2,double L3,double &theta1,double &theta2,double &theta3){ 
    /*
        x: desired x position
        y: desired y position
        z: desired z position
        L1: length of first link
        L2: length of second link
        L3: length of third link
        theta1: angle between base and first link [shoulder]
        theta2: angle between first and second links [elbow]
        theta3: angle between second link and end-effector [wrist]
    */
    // IK for shoulder calculated using inverse tangent in the (x,y) plane
    theta1 = atan2(y, x);

    // IK for wrist orientation calculated using cosine rule where the lengths
    // are L2, L3, and rxy (the distance between origin and the wrist) and where the angle between
    // L2 and L3 is theta3 and is opposite to L1.
    double rxy = sqrt(pow(x,2) + pow(y,2));
    theta3 = acos((pow(rxy,2) + pow((z-L1),2) - pow(L2,2) - pow(L3,2))/(2*L2*L3));
    theta3 = (z-L1)>0 ? theta3:-theta3;

    // IK for elbow orientation calculated by first computing the forward kinematics of the wrist
    // First term in theta2 is the angle between the x-axis and the projection of the wrist position onto the x-y plane
    // Second term in theta2 is the angle between the projection of the wrist position onto the x-y plane and the second link
    double x_wrist = cos(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    double y_wrist = sin(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
    //double z_wrist = L1 + (L2 * sin(theta2)) + (L3 * sin(theta2 + theta3));
    theta2 = atan2(y_wrist, x_wrist) - atan2(L3 * sin(theta3), L2 + (L3 * cos(theta3)));
}


///////////////////////////////////////////////////
// Second function for testing IK
///////////////////////////////////////////////////
void inverseKinematicsV2(double x, double y, double z, double L1, double L2, double L3, double &theta1, double &theta2, double &theta3){
    double d = sqrt(x*x + y*y + z*z);
    double cosTheta1 = (d*d + L1*L1 - L2*L2) / (2 * d * L1);
    theta1 = atan2(sqrt(1 - cosTheta1*cosTheta1), cosTheta1);
    double cosTheta2 = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cosTheta2*cosTheta2), cosTheta2);
    theta3 = atan2(y, x) - (theta1 + theta2);

    // Radians to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    theta3 = theta3 * 180.0 / M_PI;

    // Ensure that the joint angles are between 0 and 180 degrees
    if (theta1 < 0) 
        theta1 += 360.0;
    if (theta2 < 0)
        theta2 += 360.0;
    if (theta3 < 0)
        theta3 += 360.0;
    if (theta1 > 180.0)
        theta1 -= 180.0;
    if (theta2 > 180.0)
        theta2 -= 180.0;
    if (theta3 > 180.0)
        theta3 -= 180.0;
} 








    /*
int main(){
    Hardware::Processor::setupProcessor();
    using Hardware::Servos::Servo;
    Servo Servo1{Hardware::Processor::SERVO1_PWM};
    Servo1.enable();
    Servo Servo2{Hardware::Processor::SERVO2_PWM};
    Servo2.enable();
    Servo Servo3{Hardware::Processor::SERVO2_PWM};
    Servo3.enable();
    Servo Servo4{Hardware::Processor::SERVO3_PWM};
    Servo4.enable();
    */

    resetArmPosition(90,30,0,100);

   /*
   * S1: Shoulder, lower angles to left w.r.t it facing towards you
   * S2: Elbow, lower angles above w.r.t it facing towards you
   * S3: Wrist, lower angles below w.r.t it facing towards you
   * M4: Claw, lower angles to open w.r.t it facing towards you
   */
    //servos_init();

    /*
    //resetArmPosition(90,30,100,closeM3);
    Cy_SysLib_Delay(1000);
    gradualMovement(180, 90, Servo_3);    // open claw

    // DEFINE x,y,z location of the object
    float x = 50.0;
    float y = 0.0;
    float z = 0.0;

    // Inverse kinematics  
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    inverseKinematicsV2(x, y, z, L1, L2, L3, theta1, theta2, theta3);

    // Move arm accordingly
    moveArmForward(theta1, theta2, theta3);
    Cy_SysLib_Delay(2000);

    // Dump object into dumptruck
    dumpArmMotion();
    Cy_SysLib_Delay(1000);

    return 0;
  */
}