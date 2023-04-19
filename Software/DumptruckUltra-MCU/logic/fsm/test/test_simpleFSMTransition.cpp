#include "DumptruckUltra.hpp"
#include "fsmStates.hpp"
#include <memory>

auto main() -> int {
    using Logic::FSM::DumptruckUltra;
    // Make all states and make sure transitions between them work
    auto dumptruckFSM = std::make_unique<DumptruckUltra>();
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::INIT,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::initStateAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::DRIVE_TO_SEARCH,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::driveToSearchAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::LOCAL_SEARCH,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::localSearchAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::APPROACH,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::approachAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::PICKUP,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::pickupAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::DRIVE_TO_START,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::driveToStartAction(); });
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::DISPENSE,
        []() -> DumptruckUltra::FSMState { return Logic::FSM::dispenseAction(); });

    vTaskStartScheduler();
}