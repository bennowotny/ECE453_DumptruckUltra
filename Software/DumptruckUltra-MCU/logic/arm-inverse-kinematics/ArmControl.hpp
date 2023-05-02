#ifndef LOGIC_ARM_INVERSE_KINEMATICS_ARMCONTROL_HPP
#define LOGIC_ARM_INVERSE_KINEMATICS_ARMCONTROL_HPP

#include "hw/servos/Servo.hpp"
#include "hw/proc/proc_setup.hpp"
#include <functional>
#include <cmath>

namespace Logic {
namespace Arm {

using Hardware::Servos::Servo;

struct ArmLayout {
    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo claw;
};

class ArmControl {

    public:
        ArmControl(ArmLayout &armLayout, std::function<bool()> isClawOpen);

        void collect(float distanceForward_m);

    private:
            // Pass in ArmLayout and extract into individual Servo objects
            Hardware::Servos::Servo shoulder;
            Hardware::Servos::Servo elbow;
            Hardware::Servos::Servo wrist;
            Hardware::Servos::Servo claw;
            Hardware::Servos::Servo::SweepConfiguration sweepParams;

            void resetPosition(float currentPosition1, float currentPosition2, float currentPosition3, float currentPosition4);
            void moveArmForward(float theta1, float theta2, float theta3);
            void dumpArmMotion();
            void inverse_kinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3);

            // Arm lengths
            static constexpr float L1 = 87.0;          // Length of shoulder
            static constexpr float L2 = 50.0;          // Length of elbow
            //static constexpr float L3 = 82.0;          // Length of wrist
            static constexpr float L3 = 87.0;

            // Default angle configurations
            static constexpr float resetS1 = 90.0;          // Shoulder reset position
            static constexpr float resetS2 = 30.0;          // Elbow reset position
            static constexpr float resetS3 = 100.0;         // Wrist reset position
            static constexpr float openClaw = 70.0;         // Claw reset position [open]
            static constexpr float closeClaw = 140.0;       // Claw reset position [close]

};
} // namespace Arm
} // namespace Logic

#endif