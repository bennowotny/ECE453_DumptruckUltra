#include "FreeRTOSConfig.h"
#include "RGBLED.hpp"
#include "proc_setup.hpp"
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    // const auto blinkyR{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_RED, 0, "R")};
    // const auto blinkyG{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_GREEN, 0, "G")};
    // const auto blinkyB{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_BLUE, 0, "B")};

    // const auto rgb{std::make_unique<Hardware::RGB_LED::RGBLED>()};

    const auto blinkyTaskSetupResult{
        xTaskCreate(
            [](void *obj) {

    // Initialize LED
    

    // Toggle LED
    while (true) {
        // cyhal_gpio_toggle(Hardware::Processor::USER_RGB_RED);
        cyhal_gpio_toggle(Hardware::Processor::USER_RGB_GREEN);
        cyhal_gpio_toggle(Hardware::Processor::USER_RGB_BLUE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } },
            "Blink White",
            configMINIMAL_STACK_SIZE,
            nullptr,
            tskIDLE_PRIORITY + 1,
            nullptr)};
    CY_ASSERT(blinkyTaskSetupResult == pdPASS);

    const auto blinky{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_LED)};

    vTaskStartScheduler();

    CY_ASSERT(false); // Scheduler does not return
}