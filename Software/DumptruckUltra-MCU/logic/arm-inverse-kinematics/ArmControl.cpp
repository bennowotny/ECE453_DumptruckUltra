#include "ArmControl.hpp"

namespace Logic {
namespace Arm {

    using Hardware::Servos::Servo;


    ArmControl::ArmControl(ArmLayout &armLayout, std::function<bool()> isClawOpen) : 
        shoulder{armLayout.shoulder}, 
        elbow{armLayout.elbow}, 
        wrist{armLayout.wrist},claw{armLayout.claw} {
        shoulder.enable();
        elbow.enable();
        wrist.enable();
        claw.enable();
    }

    void ArmControl::collect(double distanceForward_m) {

        // Only consider single distance for now 
        double xForward_m = 0.0;
        double yForward_m = distanceForward_m;
        double xForward_mm = xForward_m * 1000;
        double yForward_mm = yForward_m * 1000;
        double zForward_mm = -15.0;                 // constant z position

        double theta1 = 0;
        double theta2 = 0;
        double theta3 = 0;

        // start from reset
        resetPosition(90, 30, 100, 180);

        // Returns angles in degrees and reorient
        inverse_kinematics(xForward_mm, yForward_mm, zForward_mm, theta1, theta2, theta3);
        theta1 = 180-(theta1);
        theta2 = theta2>90 ? theta2 : 180-(theta2);
        theta3 = theta3>90 ? theta3 : 180-(theta3);
        
        // Moves arm to pick up object and then to reset
        moveArmForward(theta1, theta2, theta3);

        // dump the object
        dumpArmMotion();
    }

    //////////////////////////////////
    // Private Methods
    //////////////////////////////////
    void ArmControl::resetPosition(double currentPosition1, double currentPosition2, double currentPosition3, double currentPosition4){
        double endPosition1{resetS1};
        double endPosition2{resetS2};
        double endPosition3{resetS3};
        double endPosition4{closeClaw};
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        shoulder.sweep(currentPosition1, endPosition1, sweepParams);
        elbow.sweep(currentPosition2, endPosition2, sweepParams);
        wrist.sweep(currentPosition3, endPosition3, sweepParams);
        claw.sweep(currentPosition4, endPosition4, sweepParams);
    }


    void ArmControl::moveArmForward(double theta1, double theta2, double theta3){

        // Assumes that the arm is currently in a reset position
        // Set into reset as pre-requisite check
        shoulder.setPosition(resetS1);
        elbow.setPosition(resetS2);
        wrist.setPosition(resetS3);
        claw.setPosition(openClaw);   // open claw

        // (theta1, theta2, theta3) should be in degrees
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        shoulder.sweep(resetS1, theta1, sweepParams);
        elbow.sweep(resetS2, theta2, sweepParams);
        wrist.sweep(resetS3, theta3, sweepParams);
        Cy_SysLib_Delay(1000);
        claw.sweep(openClaw, closeClaw, sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset to starting position
        resetPosition(theta1, theta2, theta3, closeClaw);
    }


    void ArmControl::dumpArmMotion(){
        // Assumes that the arm is currently in a reset position
        // Set into reset as pre-requisite check
        shoulder.setPosition(resetS1);
        elbow.setPosition(resetS2);
        wrist.setPosition(resetS3);
        claw.setPosition(openClaw);   // open claw

        // 0 degrees: farthest point elbow can go back 
        // 180 degrees: farthest point wrist can go back
        Hardware::Servos::Servo::SweepConfiguration sweepParams;
        elbow.sweep(resetS2, 0, sweepParams);
        wrist.sweep(resetS3, 180, sweepParams);
        claw.sweep(closeClaw, openClaw, sweepParams);
        Cy_SysLib_Delay(2000);
        claw.sweep(openClaw, closeClaw, sweepParams);
        Cy_SysLib_Delay(1000);

        // Reset Arm Position
        resetPosition(resetS1, 0, 180, closeClaw);
    }



    void ArmControl::inverse_kinematics(double x, double y, double z, double &theta1, double &theta2, double &theta3){
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
} // namespace Logic
