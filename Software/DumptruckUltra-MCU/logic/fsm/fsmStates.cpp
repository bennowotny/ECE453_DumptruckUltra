#include "fsmStates.hpp"
#include "DistanceSensor.hpp"
#include "DrivingAlgorithm.hpp"
#include "DumptruckUltra.hpp"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "pressure_sensor.hpp"
#include "proc_setup.hpp"
#include "projdefs.h"
#include <memory>

namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing init state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

// TODO: Take in driving algo as param
// Make random number 1-10 for x and y coordinates
// start() allows for repeated calls to getStatus()
auto driveToSearchAction(const std::shared_ptr<Logic::DrivingAlgorithm::DrivingAlgorithm> &drivingAlg) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing driveToSearch state\n");
    float randX = 1 + rand() % 10;
    float randY = 1 + rand() % 10;
    printf("%d", drivingAlg->getStatus());
    drivingAlg->loadNewTarget({.x = randX, .y = randY, .heading = 0});
    printf("%d", drivingAlg->getStatus());
    drivingAlg->start();
    printf("%d", drivingAlg->getStatus());
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::LOCAL_SEARCH;
}

auto localSearchAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing localSearch state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::APPROACH;
}

auto approachAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing approach state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::PICKUP;
}

// TODO: Uses Camera Reading
auto pickupAction(const std::shared_ptr<Hardware::DistanceSensor::DistanceSensor> &distSensor, std::unique_ptr<Logic::Arm::ArmControl> arm, const std::shared_ptr<Hardware::PressureSensor::PressureSensor> &pressureSensor) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing pickup state\n");
    float pickupDistance = 20.0;
    // Distance sensor reading and camera recognize object is at right distance
    if (distSensor->getDistanceMeters() != pickupDistance) {
        return DumptruckUltra::FSMState::APPROACH;
    }
    return DumptruckUltra::FSMState::DRIVE_TO_START;
}

auto driveToStartAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing driveToStart state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DISPENSE;
}

auto dispenseAction(const std::shared_ptr<Hardware::Servos::Servo> &dispenserServo) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing dispense state\n");
    // TODO: test for dispenser servo value, assume 22 for now
    float dispenserOpen = 22.0;
    float dispenserClose = 0.0;
    // Open Dispenser
    dispenserServo->setPosition(dispenserOpen);
    vTaskDelay(pdMS_TO_TICKS(500));
    // Close Dispenser
    dispenserServo->setPosition(dispenserClose);
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

} // namespace FSM
} // namespace Logic