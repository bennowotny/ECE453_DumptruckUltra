#include "DumptruckUltra.hpp"
#include "fsmStates.hpp"
#include "proc_setup.hpp"
#include <memory>

Logic::FSM::DumptruckUltra::FSMState blinkyAction() {
    return Logic::FSM::DumptruckUltra::FSMState::INIT;
}

auto main() -> int {
    using Logic::FSM::DumptruckUltra;
    // Instantiate blinky object

    const auto blinkyR{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(P9_4, 0, "R")};
    const auto blinkyG{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(P9_5, 0, "G")};
    const auto blinkyB{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(P9_6, 0, "B")};

    // Make all states and make sure transitions between them work
    auto dumptruckFSM = std::make_unique<DumptruckUltra>();
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::INIT,
        [blinkyR, blinkyG, blinkyB]() -> DumptruckUltra::FSMState { return blinkyAction(); });
}