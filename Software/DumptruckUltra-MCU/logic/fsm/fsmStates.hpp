#pragma once

#include "DumptruckUltra.hpp"
#include "dispenser/Dispenser.hpp"
#include "hw/distance_sensor/DistanceSensor.hpp"
#include "hw/pressure_sensor/pressure_sensor.hpp"
#include "hw/servos/Servo.hpp"
#include "logic/arm-inverse-kinematics/ArmControl.hpp"
#include "logic/vision/ObjectDetector.hpp"


namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState;
auto driveToSearchAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState;
auto localSearchAction(DrivingAlgorithm::DriveMotorLayout &driveMotors, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState;
auto approachAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg, Logic::Vision::ObjectDetector &vision) -> DumptruckUltra::FSMState;
auto pickupAction(Logic::Arm::ArmControl &arm, Logic::Vision::ObjectDetector &vision, Logic::DeadReckoning::DeadReckoning &deadReckoning, Logic::Dispenser::Dispenser &dispenser) -> DumptruckUltra::FSMState;
auto driveToStartAction(Logic::DrivingAlgorithm::DrivingAlgorithm &drivingAlg) -> DumptruckUltra::FSMState;
auto dispenseAction(Logic::Dispenser::Dispenser &dispenser) -> DumptruckUltra::FSMState;

} // namespace FSM
} // namespace Logic