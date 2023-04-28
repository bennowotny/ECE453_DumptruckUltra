#pragma once

#include "DumptruckUltra.hpp"
#include "hw/distance_sensor/DistanceSensor.hpp"
#include "hw/pressure_sensor/pressure_sensor.hpp"
#include "hw/servos/Servo.hpp"
#include "logic/arm-inverse-kinematics/ArmControl.hpp"

namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState;
auto driveToSearchAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState;
auto localSearchAction() -> DumptruckUltra::FSMState;
auto approachAction() -> DumptruckUltra::FSMState;
auto pickupAction(Hardware::DistanceSensor::DistanceSensor &distSensor, Logic::Arm::ArmControl &arm, Hardware::PressureSensor::PressureSensor &pressureSensor) -> DumptruckUltra::FSMState;
auto driveToStartAction() -> DumptruckUltra::FSMState;
auto dispenseAction(Hardware::Servos::Servo &dispenserServo) -> DumptruckUltra::FSMState;

} // namespace FSM
} // namespace Logic