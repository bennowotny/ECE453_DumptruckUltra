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

auto initStateAction(const std::function<void()> &setup) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing init state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    setup();
    vTaskDelay(pdMS_TO_TICKS(10'000));
    srand(xTaskGetTickCount());
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

auto driveToSearchAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState {
    // // printf("%s", "Doing driveToSearch state\n");
    // const float randX{10 * static_cast<float>(rand()) / RAND_MAX};
    // const float randY{10 * static_cast<float>(rand()) / RAND_MAX};
    vTaskDelay(pdMS_TO_TICKS(1000));

    // printf("%d", drivingAlg->getStatus());
    drivingAlg.loadNewTarget({.x = 2, .y = 1, .heading = 0});
    // printf("%d", drivingAlg->getStatus());
    drivingAlg.start();
    while (drivingAlg.getStatus() != DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // printf("%d", drivingAlg->getStatus());
    return DumptruckUltra::FSMState::LOCAL_SEARCH;
}

auto localSearchAction(const std::function<void(Logic::DrivingAlgorithm::MotorSpeeds)> &motorControl, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing localSearch state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    motorControl({.leftPower = -0.15, .rightPower = 0.15});

    while (!vision.detectedObject()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    motorControl({.leftPower = 0, .rightPower = 0});

    return DumptruckUltra::FSMState::APPROACH;
}

auto approachAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing approachAction state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    const auto target{vision.currentObjectLocation()};

    drivingAlg.loadNewTarget(target);
    // printf("%d", drivingAlg->getStatus());
    drivingAlg.start();
    while (drivingAlg.getStatus() != DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return DumptruckUltra::FSMState::PICKUP;
}

auto pickupAction(Logic::Arm::ArmControl &arm, Logic::Vision::ObjectDetector &vision, Logic::DeadReckoning::DeadReckoning &deadReckoning, Logic::Dispenser::Dispenser &dispenser) -> DumptruckUltra::FSMState {
    // printf("%s", "Doing pickup state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    const auto target{vision.currentObjectLocation()};
    const auto currentPosition{deadReckoning.getCurrentPose()};
    const auto distance{std::sqrt(std::pow(target.x - currentPosition.x, 2) + std::pow(target.y - currentPosition.y, 2))};

    arm.collect(distance);

    dispenser.placeObject();
    return dispenser.full() ? DumptruckUltra::FSMState::DRIVE_TO_START : DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

auto driveToStartAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState {
    vTaskDelay(pdMS_TO_TICKS(1000));
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
    vTaskDelay(pdMS_TO_TICKS(1000));
    dispenser.open();
    vTaskDelay(pdMS_TO_TICKS(5000));
    dispenser.close();
    dispenser.clear();
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

} // namespace FSM
} // namespace Logic