#include "fsmStates.hpp"
#include "DeadReckoning.hpp"
#include "DistanceSensor.hpp"
#include "DrivingAlgorithm.hpp"
#include "DumptruckUltra.hpp"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "dispenser/Dispenser.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "pressure_sensor.hpp"
#include "projdefs.h"
#include "vision/ObjectDetector.hpp"
#include <cmath>
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

auto localSearchAction(DrivingAlgorithm::DriveMotorLayout &driveMotors, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing localSearch state\n");

    driveMotors.leftMotor.enable();
    driveMotors.rightMotor.enable();

    driveMotors.leftMotor.setPower(-0.3);
    driveMotors.rightMotor.setPower(0.3);

    while (!vision.detectedObject()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    driveMotors.leftMotor.disable();
    driveMotors.rightMotor.disable();

    return DumptruckUltra::FSMState::APPROACH;
}

auto approachAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState {

    const auto target{vision.currentObjectLocation()};

    drivingAlg.loadNewTarget(target);
    // printf("%d", drivingAlg->getStatus());
    drivingAlg.start();
    while (drivingAlg.getStatus() != DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return DumptruckUltra::FSMState::PICKUP;
}

// TODO: Uses Camera Reading
auto pickupAction(Logic::Arm::ArmControl &arm, Logic::Vision::ObjectDetector &vision, Logic::DeadReckoning::DeadReckoning &deadReckoning, Logic::Dispenser::Dispenser &dispenser) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing pickup state\n");
    const auto target{vision.currentObjectLocation()};
    const auto currentPosition{deadReckoning.getCurrentPose()};
    const auto distance{std::sqrt(std::pow(target.x - currentPosition.x, 2) + std::pow(target.y - currentPosition.y, 2))};

    arm.collect(distance);

    dispenser.placeObject();
    return dispenser.full() ? DumptruckUltra::FSMState::DRIVE_TO_START : DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

auto driveToStartAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState {
    drivingAlg.loadNewTarget({.x = 0, .y = 0, .heading = 0});
    // printf("%d", drivingAlg->getStatus());
    drivingAlg.start();
    while (drivingAlg.getStatus() != DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return DumptruckUltra::FSMState::DISPENSE;
}

auto dispenseAction(Logic::Dispenser::Dispenser &dispenser) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing dispense state\n");
    // TODO: test for dispenser servo value, assume 22 for now
    dispenser.open();
    vTaskDelay(pdMS_TO_TICKS(5000));
    dispenser.close();
    dispenser.clear();
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

} // namespace FSM
} // namespace Logic