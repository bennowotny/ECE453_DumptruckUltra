#include "DeadReckoning.hpp"
#include "DistanceSensor.hpp"
#include "DrivingAlgorithm.hpp"
#include "DumptruckUltra.hpp"
#include "Positioning.hpp"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "dispenser/Dispenser.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "fsmStates.hpp"
#include "pressure_sensor.hpp"
#include "projdefs.h"
#include "vision/ObjectDetector.hpp"
#include <cmath>
#include <memory>
#include <random>

namespace Logic {
namespace FSMDemo {

auto initStateAction(const std::function<void()> &setup) -> FSM::DumptruckUltra::FSMState {
    // printf("%s", "Doing init state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    setup();
    vTaskDelay(pdMS_TO_TICKS(20'000));
    srand(xTaskGetTickCount());
    return FSM::DumptruckUltra::FSMState::PICKUP;
}

auto pickupAction(Logic::Arm::ArmControl &arm) -> FSM::DumptruckUltra::FSMState {
    // printf("%s", "Doing pickup state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    // const Logic::DeadReckoning::Pose2D target = {.x = 0, .y = 0.05, .heading = 0};
    // const auto currentPosition{deadReckoning.getCurrentPose()};
    // const auto distance{std::sqrt(std::pow(target.x - currentPosition.x, 2) + std::pow(target.y - currentPosition.y, 2))};

    // arm.collect(0.05);

    return FSM::DumptruckUltra::FSMState::DISPENSE;
}

auto dispenseAction(Logic::Dispenser::Dispenser &dispenser) -> FSM::DumptruckUltra::FSMState {
    // printf("%s", "Doing dispense state\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    dispenser.open();
    vTaskDelay(pdMS_TO_TICKS(5000));
    dispenser.close();
    dispenser.clear();
    return FSM::DumptruckUltra::FSMState::INIT;
}

} // namespace FSMDemo
} // namespace Logic