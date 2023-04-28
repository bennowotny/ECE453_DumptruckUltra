#include "fsmStates.hpp"
#include "DistanceSensor.hpp"
#include "DrivingAlgorithm.hpp"
#include "DumptruckUltra.hpp"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "pressure_sensor.hpp"
#include "projdefs.h"
#include <memory>
#include <random>

namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState {
    // printf("%s", "Doing init state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

// TODO: Take in driving algo as param
// Make random number 1-10 for x and y coordinates
// start() allows for repeated calls to getStatus()
auto driveToSearchAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing driveToSearch state\n");
    std::random_device randomSource{};
    std::default_random_engine randomEngine{randomSource()};
    std::uniform_real_distribution<float> distribution{0, 10};
    const float randX{distribution(randomEngine)};
    const float randY{distribution(randomEngine)};

    // printf("%d", drivingAlg->getStatus());
    drivingAlg.loadNewTarget({.x = randX, .y = randY, .heading = 0});
    // printf("%d", drivingAlg->getStatus());
    drivingAlg.start();
    while (drivingAlg.getStatus() != DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // printf("%d", drivingAlg->getStatus());
    return DumptruckUltra::FSMState::LOCAL_SEARCH;
}

auto localSearchAction() -> DumptruckUltra::FSMState {
    // printf("%s", "Doing localSearch state\n");

    return DumptruckUltra::FSMState::APPROACH;
}

auto approachAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing approach state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::PICKUP;
}

// TODO: Uses Camera Reading
auto pickupAction(Hardware::DistanceSensor::DistanceSensor &distSensor, Logic::Arm::ArmControl &arm, Hardware::PressureSensor::PressureSensor &pressureSensor) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing pickup state\n");
    float pickupDistance = 20.0;
    // Distance sensor reading and camera recognize object is at right distance
    if (distSensor.getDistanceMeters() != pickupDistance) {
        return DumptruckUltra::FSMState::APPROACH;
    }
    return DumptruckUltra::FSMState::DRIVE_TO_START;
}

auto driveToStartAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing driveToStart state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DISPENSE;
}

auto dispenseAction(Hardware::Servos::Servo &dispenserServo) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing dispense state\n");
    // TODO: test for dispenser servo value, assume 22 for now
    float dispenserOpen = 22.0;
    float dispenserClose = 0.0;
    // Open Dispenser
    dispenserServo.setPosition(dispenserOpen);
    vTaskDelay(pdMS_TO_TICKS(500));
    // Close Dispenser
    dispenserServo.setPosition(dispenserClose);
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

} // namespace FSM
} // namespace Logic