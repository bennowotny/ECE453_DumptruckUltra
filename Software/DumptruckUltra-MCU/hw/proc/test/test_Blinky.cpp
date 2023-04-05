#include "proc_setup.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    const Hardware::Processor::FreeRTOSBlinky blinky{P5_5};

    vTaskStartScheduler();

    CY_ASSERT(false); // Scheduler does not return
}