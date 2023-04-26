#include "proc_setup.hpp"
#include "cy_result.h"
#include "cybsp.h"
#include "cyhal_system.h"
#include "projdefs.h"
#include <string>

namespace Hardware {
namespace Processor {
void setupProcessor() {

    /* Initialize the device and board peripherals */
    const auto result{cybsp_init()};
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
}

char const *const FreeRTOSBlinky::BLINKY_TASK_NAME{"Blinky"};

FreeRTOSBlinky::FreeRTOSBlinky(cyhal_gpio_t blinkyPin, uint32_t delayMs, const char *taskName) : blinkyPin{blinkyPin},
                                                                                                 delayMs{delayMs} {
    const auto blinkyTaskSetupResult{
        xTaskCreate(
            [](void *obj) { static_cast<FreeRTOSBlinky *>(obj)->ledTask(); },
            taskName,
            BLINKY_STACK_SIZE,
            this,
            BLINKY_PRIORITY,
            nullptr)};
    CY_ASSERT(blinkyTaskSetupResult == pdPASS);
}

void FreeRTOSBlinky::ledTask() {

    vTaskDelay(pdMS_TO_TICKS(delayMs));

    // Initialize LED
    const auto res{cyhal_gpio_init(blinkyPin, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)};
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Toggle LED
    while (true) {
        cyhal_gpio_toggle(blinkyPin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

} // namespace Processor
} // namespace Hardware