#include "DeadReckoning.hpp"
#include "proc_setup.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    Logic::DeadReckoning::DeadReckoning uut{};

    // More testing?
}