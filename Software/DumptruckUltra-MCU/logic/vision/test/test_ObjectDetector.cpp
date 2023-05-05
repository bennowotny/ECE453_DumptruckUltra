#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "cyhal_system.h"
#include "logic/vision/ObjectDetector.hpp"
#include "proc_setup.hpp"

int main(void) {
    Hardware::Processor::setupProcessor();
    using Hardware::Servos::Servo;
    using Logic::Arm::ArmControl;
    using Logic::Arm::ArmLayout;
    Logic::Vision::ObjectDetector vision;

    ArmLayout arm{
        .shoulder{Servo{Hardware::Processor::SERVO1_PWM}},
        .elbow{Servo{Hardware::Processor::SERVO2_PWM}},
        .wrist{Servo{Hardware::Processor::SERVO3_PWM}},
        .claw{Servo{Hardware::Processor::SERVO4_PWM}}};

    ArmControl uut{
        arm, []() -> bool { return true; }};

    __enable_irq();

    cyhal_system_delay_ms(15000); // wait 15 seconds for Pi to start
    while (true) {
        if (vision.detectedObject()) {
            auto pose = vision.currentObjectLocation();
            // float x = (float)pose.x / 1000.0;
            // float y = (float)pose.y / 1000.0;

            uut.collect(pose.y, pose.x);
            vision.resetUARTBuffer();
        }
        cyhal_system_delay_ms(100);
    }
}