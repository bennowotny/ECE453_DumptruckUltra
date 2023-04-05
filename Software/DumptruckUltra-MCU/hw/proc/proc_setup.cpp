#include "proc_setup.hpp"
#include "cy_result.h"
#include "cybsp.h"

namespace Hardware {
namespace Processor {
void setupProcessor() {

    /* Initialize the device and board peripherals */
    const auto result{cybsp_init()};
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
}

FreeRTOSBlinky::FreeRTOSBlinky() {
    const auto drivingTaskSetupResult{
        xTaskCreate(
            [](void *obj) { FreeRTOSBlinky::ledTask(); },
            BLINKY_TASK_NAME,
            BLINKY_STACK_SIZE,
            nullptr,
            BLINKY_PRIORITY,
            nullptr)};
    CY_ASSERT(drivingTaskSetupResult == pdPASS);
}

void FreeRTOSBlinky::ledTask() {

    // Initialize LED
    const auto res{cyhal_gpio_init(P5_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)};
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Toggle LED
    while (true) {
        cyhal_gpio_toggle(P5_5);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

} // namespace Processor
} // namespace Hardware