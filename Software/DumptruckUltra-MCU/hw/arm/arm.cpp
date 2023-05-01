//#include "Servo.hpp"
#include "Arm.hpp"
#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"
#include <cmath>


namespace Hardware {
namespace Arm {
    Arm::Arm()
        : Servo1{Hardware::Processor::SERVO1_PWM}, Servo2{Hardware::Processor::SERVO2_PWM}, Servo3{Hardware::Processor::SERVO2_PWM}, Servo4{Hardware::Processor::SERVO3_PWM} {
        Servo1.enable();
        Servo2.enable();
        Servo3.enable();
        Servo4.enable();
    }

    void Arm::resetPosition(float currentPosition1, float currentPosition2, float currentPosition3, float currentPosition4){
        float endPosition1{resetS1};
        float endPosition2{resetS2};
        float endPosition3{resetS3};
        float endPosition4{closeClaw};
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        Servo1.sweep(currentPosition1, endPosition1, sweepParams);
        Servo2.sweep(currentPosition2, endPosition2, sweepParams);
        Servo3.sweep(currentPosition3, endPosition3, sweepParams);
        Servo4.sweep(currentPosition4, endPosition4, sweepParams);
    }


    void Arm::moveArmForward(float theta1, float theta2, float theta3){

        // Assumes that the arm is currently in a reset position
        // Set into reset as pre-requisite check
        Servo1.setPosition(resetS1);
        Servo2.setPosition(resetS2);
        Servo3.setPosition(resetS3);
        Servo4.setPosition(openClaw);   // open claw

        // (theta1, theta2, theta3) should be in degrees
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        Servo1.sweep(resetS1, theta1, sweepParams);
        Servo2.sweep(resetS2, theta2, sweepParams);
        Servo3.sweep(resetS3, theta3, sweepParams);
        Cy_SysLib_Delay(1000);
        Servo4.sweep(openClaw, closeClaw, sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset to starting position
        resetPosition(theta1, theta2, theta3, closeClaw);
    }


    void Arm::dumpArmMotion(){
        // Assumes that the arm is currently in a reset position
        // Set into reset as pre-requisite check
        Servo1.setPosition(resetS1);
        Servo2.setPosition(resetS2);
        Servo3.setPosition(resetS3);
        Servo4.setPosition(openClaw);   // open claw

        // Farthest point the arm can go back
        int reverseS2 = 0;
        int reverseS3 = 180;

        // Keep shoulder constant, move elbow, wrist, and claw
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        Servo2.sweep(resetS2, reverseS2, sweepParams);
        Servo3.sweep(resetS3, reverseS3, sweepParams);
        Servo4.sweep(closeClaw, openClaw, sweepParams);
        Cy_SysLib_Delay(2000);
        Servo4.sweep(openClaw, closeClaw, sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset Arm Position
        resetPosition(resetS1, reverseS2, reverseS3, closeClaw);
    }



    void Arm::inverse_kinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3){
        /********************************************
            L1: length of first link        [Given]
            L2: length of second link       [Given]
            L3: length of third link        [Given]
            x: desired x position
            y: desired y position
            z: desired z position
            theta1: angle between base and first link [shoulder]
            theta2: angle between first and second links [elbow]
            theta3: angle between second link and end-effector [wrist]
        *********************************************/
        // Shoulder angle
        theta1 = atan2(y, x);

        // Wrist angle
        double rxy = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        theta3 = acos((pow(rxy,2) - pow(L2,2) - pow(L3,2))/(2*L2*L3));
        theta3 = (z-L1)>0 ? theta3:-theta3;

        // Elbow angle
        double x_wrist = cos(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
        double y_wrist = sin(theta1) * ((L2 * cos(theta2)) + (L3 * cos(theta2 + theta3)));
        theta2 = atan2(y_wrist, x_wrist) - atan2(L3 * sin(theta3), L2 + (L3 * cos(theta3)));

        // Return in degrees and between 0 and 180
        theta1 = theta1 * 180 / M_PI;
        theta2 = theta2 * 180 / M_PI;
        theta3 = theta3 * 180 / M_PI;
        if(theta1 < 0) theta1 += 180;
        if(theta2 < 0) theta2 += 180;
        if(theta3 < 0) theta3 += 180;
        if(theta1 > 180) theta1 -= 180;
        if(theta2 > 180) theta2 -= 180;
        if(theta3 > 180) theta3 -= 180;
    }
} // namespace Arm
} // namespace Hardware