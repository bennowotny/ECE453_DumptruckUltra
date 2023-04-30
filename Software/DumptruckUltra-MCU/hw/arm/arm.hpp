
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
            void compute_ik(float x, float y, float z, float &theta1, float &theta2, float &theta3);
            void moveArmForward(float theta1, float theta2, float theta3);

        private:
            Hardware::Servos::Servo Servo1;
            Hardware::Servos::Servo Servo2;
            Hardware::Servos::Servo Servo3;
            Hardware::Servos::Servo Servo4;
    };
};
};
#endif