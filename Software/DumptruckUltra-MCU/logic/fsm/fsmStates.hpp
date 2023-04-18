#pragma once

#include "DumptruckUltra.hpp"

namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState;
auto driveToSearchAction() -> DumptruckUltra::FSMState;
auto localSearchAction() -> DumptruckUltra::FSMState;
auto approachAction() -> DumptruckUltra::FSMState;
auto pickupAction() -> DumptruckUltra::FSMState;
auto driveToStartAction() -> DumptruckUltra::FSMState;
auto dispenseAction() -> DumptruckUltra::FSMState;

} // namespace FSM
} // namespace Logic