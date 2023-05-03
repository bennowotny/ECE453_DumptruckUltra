#pragma once

#include "DumptruckUltra.hpp"
#include "dispenser/Dispenser.hpp"
#include "driving-algorithm/DrivingAlgorithm.hpp"
#include "logic/arm-inverse-kinematics/ArmControl.hpp"
#include "vision/ObjectDetector.hpp"

namespace Logic {
namespace FSMDemo {

auto initStateAction(const std::function<void()> &setup) -> FSM::DumptruckUltra::FSMState;
auto pickupAction(Logic::Arm::ArmControl &arm) -> FSM::DumptruckUltra::FSMState;
auto dispenseAction(Logic::Dispenser::Dispenser &dispenser) -> FSM::DumptruckUltra::FSMState;

} // namespace FSMDemo
} // namespace Logic