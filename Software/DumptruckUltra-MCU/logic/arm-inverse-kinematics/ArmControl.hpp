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

        void collect(double distanceForward_m);

    private:
            // Pass in ArmLayout and extract into individual Servo objects
            Hardware::Servos::Servo shoulder;
            Hardware::Servos::Servo elbow;
            Hardware::Servos::Servo wrist;
            Hardware::Servos::Servo claw;

            void resetPosition(double currentPosition1, double currentPosition2, double currentPosition3, double currentPosition4);
            void moveArmForward(double theta1, double theta2, double theta3);
            void dumpArmMotion();
            void inverse_kinematics(double x, double y, double z, double &theta1, double &theta2, double &theta3);

            // Arm lengths
            static constexpr double L1 = 87.0;          // Length of shoulder
            static constexpr double L2 = 50.0;          // Length of elbow
            static constexpr double L3 = 82.0;          // Length of wrist

            // Default angle configurations
            static constexpr double resetS1 = 90.0;          // Shoulder reset position
            static constexpr double resetS2 = 30.0;          // Elbow reset position
            static constexpr double resetS3 = 100.0;         // Wrist reset position
            static constexpr double openClaw = 80.0;         // Claw reset position [open]
            static constexpr double closeClaw = 180.0;       // Claw reset position [close]

};
} // namespace Arm
} // namespace Logic

#endif