
#include "Motor.hpp"
#include "cyhal_psoc6_01_43_smt.h"
#include "logic/dead-reckoning/DeadReckoning.hpp"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
#include "proc_setup.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    Logic::DrivingAlgorithm::DriveMotorLayout layout{
        .leftMotor = Hardware::Motors::Motor{{.forwardPin = P9_3, .backwardPin = P7_1}, Hardware::Motors::MotorDirection::FORWARD}, // TODO: Pick pins
        .rightMotor = Hardware::Motors::Motor{{.forwardPin = P12_6, .backwardPin = P12_7}, Hardware::Motors::MotorDirection::REVERSE}};
    Logic::DrivingAlgorithm::DrivingAlgorithm uut{
        layout,
        []() -> float { return 0.1; },                 // Nothing in the way
        []() -> Logic::DeadReckoning::Pose2D { return {// Don't move
                                                       .x = 0,
                                                       .y = 0,
                                                       .heading = 0}; }};

    // FOR TESTER: Change this line to get different behavior out of the driving algorithm
    // Currently: go forward
    uut.loadNewTarget({.x = 1, .y = 0, .heading = 0});

    uut.start();

    // Also run blinky so we know if the RTOS is having a problem
    const Hardware::Processor::FreeRTOSBlinky blinky{};

    vTaskStartScheduler();

    CY_ASSERT(false); // Do not return from the scheduler
}