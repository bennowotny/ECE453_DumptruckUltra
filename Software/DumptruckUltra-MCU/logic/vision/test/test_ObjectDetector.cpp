#include "logic/vision/ObjectDetector.hpp"
#include "cyhal_system.h"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "proc_setup.hpp"

int main(void){
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


    cyhal_system_delay_ms(15000);       // wait 15 seconds for Pi to start
    while (true) {
        if (vision.detectedObject()) {
            //auto pose = vision.currentObjectLocation();
            //float x = (float)pose.x;
            //float y = (float)pose.y;

            //float distanceForward_m = (float)pose.x*1000;
            float distanceForward_m = 0.05;
            uut.collect(distanceForward_m);

            //printf("x: %f, y: %f\n", pose.x, pose.y);
        }
        //cyhal_system_delay_ms(100);
    }
}