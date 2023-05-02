#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "proc_setup.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::Servos::Servo;
    using Logic::Arm::ArmControl;
    using Logic::Arm::ArmLayout;

    ArmLayout arm{
        .shoulder{Servo{Hardware::Processor::SERVO1_PWM}},
        .elbow{Servo{Hardware::Processor::SERVO2_PWM}},
        .wrist{Servo{Hardware::Processor::SERVO3_PWM}},
        .claw{Servo{Hardware::Processor::SERVO4_PWM}}};

    ArmControl uut{
        arm, []() -> bool { return true; }};

    while (true) {
        // TODO: Write test
        float distanceForward_m = 0.05;
        uut.collect(distanceForward_m);
    }
}
