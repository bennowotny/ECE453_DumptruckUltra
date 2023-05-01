
#ifndef HW_ARM_HPP_
#define HW_ARM_HPP_

#include "Servo.hpp"
#include "cyhal_hw_types.h"
#include "hw/proc/proc_setup.hpp"

namespace Hardware {
namespace Arm {
    class Arm{ 
        public:
            explicit Arm();
            void resetPosition(float currentPosition1, float currentPosition2, float currentPosition3, float currentPosition4);
            void moveArmForward(float theta1, float theta2, float theta3);
            void dumpArmMotion();
            void inverse_kinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3);

        private:
            Hardware::Servos::Servo Servo1;
            Hardware::Servos::Servo Servo2;
            Hardware::Servos::Servo Servo3;
            Hardware::Servos::Servo Servo4;

            static constexpr double L1 = 87.0;          // Length of shoulder
            static constexpr double L2 = 50.0;          // Length of elbow
            static constexpr double L3 = 82.0;          // Length of wrist

            static constexpr int resetS1 = 90;          // Shoulder reset position
            static constexpr int resetS2 = 30;          // Elbow reset position
            static constexpr int resetS3 = 100;         // Wrist reset position
            static constexpr int openClaw = 80;         // Claw reset position [open]
            static constexpr int closeClaw = 180;       // Claw reset position [close]
    };
};
};
#endif