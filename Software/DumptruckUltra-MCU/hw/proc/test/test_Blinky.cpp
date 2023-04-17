#include "proc_setup.hpp"
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    const auto blinky{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(P7_1)};

    vTaskStartScheduler();

    CY_ASSERT(false); // Scheduler does not return
}