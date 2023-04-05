#include "proc_setup.hpp"
#include "cybsp.h"

namespace Hardware {
namespace Processor {
void setupProcessor() {

    /* Initialize the device and board peripherals */
    const auto result{cybsp_init()};
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
}

FreeRTOSBlinky::FreeRTOSBlinky(cyhal_gpio_t blinkyPin) : blinkyPin{blinkyPin} {
    const auto blinkyTaskSetupResult{
        xTaskCreate(
            [](void *obj) { FreeRTOSBlinky::ledTask(static_cast<FreeRTOSBlinky *>(obj)); },
            BLINKY_TASK_NAME,
            BLINKY_STACK_SIZE,
            this,
            BLINKY_PRIORITY,
            nullptr)};
    CY_ASSERT(blinkyTaskSetupResult == pdPASS);
}

void FreeRTOSBlinky::ledTask(FreeRTOSBlinky *obj) {

    // Initialize LED
    const auto res{cyhal_gpio_init(obj->blinkyPin, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)};
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Toggle LED
    while (true) {
        cyhal_gpio_toggle(obj->blinkyPin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

} // namespace Processor
} // namespace Hardware