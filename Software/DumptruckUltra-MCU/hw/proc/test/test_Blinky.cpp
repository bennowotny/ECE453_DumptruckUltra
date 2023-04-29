#include "FreeRTOSConfig.h"
#include "proc_setup.hpp"
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    // const auto blinkyR{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_RED, 0, "R")};
    // const auto blinkyG{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_GREEN, 0, "G")};
    // const auto blinkyB{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_RGB_BLUE, 0, "B")};

    const auto blinkyTaskSetupResult{
        xTaskCreate(
            [](void *obj) {

    // Initialize LED
    const auto resr{cyhal_gpio_init(Hardware::Processor::USER_RGB_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true)};
    CY_ASSERT(resr == CY_RSLT_SUCCESS);
    const auto resg{cyhal_gpio_init(Hardware::Processor::USER_RGB_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true)};
    CY_ASSERT(resg == CY_RSLT_SUCCESS);
    const auto resb{cyhal_gpio_init(Hardware::Processor::USER_RGB_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true)};
    CY_ASSERT(resb == CY_RSLT_SUCCESS);

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