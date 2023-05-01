#include "Servo.hpp"
#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"

#include "Arm.hpp"

auto main() -> int {

    Hardware::Processor::setupProcessor();

    Hardware::Arm::Arm lombard{};

    // object is forward and to the left of the robot
    float x = 50;
    float y = 50;
    float z = -15;

    float theta1 = 0;
    float theta2 = 0;
    float theta3 = 0;
    lombard.resetPosition(90, 30, 100, 180);
    lombard.inverse_kinematics(x, y, z, theta1, theta2, theta3);
    lombard.moveArmForward(theta1, theta2, theta3);
    lombard.dumpArmMotion();
}