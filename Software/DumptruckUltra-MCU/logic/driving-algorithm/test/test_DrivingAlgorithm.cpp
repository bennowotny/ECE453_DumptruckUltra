
#include "Motor.hpp"
#include "cyhal_psoc6_01_43_smt.h"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
#include "proc_setup.hpp"
#include <cmath>
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    Logic::DrivingAlgorithm::DriveMotorLayout layout{
        .leftMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M1_FORWARD, .backwardPin = Hardware::Processor::M1_BACKWARD}, Hardware::Motors::MotorDirection::FORWARD},
        .rightMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M2_FORWARD, .backwardPin = Hardware::Processor::M2_BACKWARD}, Hardware::Motors::MotorDirection::REVERSE}};

    const auto uut{std::make_unique<Logic::DrivingAlgorithm::DrivingAlgorithm>(
        layout,
        // FOR TESTER: Change this line to simulate obstacles
        []() -> float { constexpr float SIMULATED_DISTANCE_METERS{99}; return SIMULATED_DISTANCE_METERS; },
        // FOR TESTER: Change this line to simulate different current positions
        []() -> Logic::DeadReckoning::Pose2D { return {// Don't move
                                                       .x = 0,
                                                       .y = 0,
                                                       .heading = 0}; })};

    // FOR TESTER: Change this line to set the target of the driving algorithm
    uut->loadNewTarget({.x = 100, .y = 0, .heading = 0});

    uut->start();

    // Also run blinky so we know if the RTOS is having a problem
    const auto blinky{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_LED)};

    vTaskStartScheduler();

    CY_ASSERT(false); // Do not return from the scheduler
}