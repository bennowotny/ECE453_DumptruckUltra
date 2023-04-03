#include "cy_utils.h"
#include "cyhal_gpio_impl.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "hw/proc/proc_setup.hpp"

#include "FreeRTOS.h"
#include "task.h"

void ledTask(void *pvParameters) {
    // Simple task that toggle an LED
    // FIXME: Change to work with our PCB
    (void)pvParameters;

    // Initialize LED
    cy_rslt_t res = cyhal_gpio_init(P5_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    if (res != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    // Toggle LED
    while (true) {
        cyhal_gpio_toggle(P5_5);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {

    // Create simple task that toggles the onboard dev board LED
    // FIXME: Change to work with our PCB
    TaskHandle_t taskHandle = nullptr;
    BaseType_t ret = xTaskCreate(
        ledTask,
        "ledTask",
        1000,
        nullptr,
        tskIDLE_PRIORITY,
        &taskHandle);

    if (ret != pdPASS) {
        CY_ASSERT(0);
    }

    vTaskStartScheduler();

    CY_ASSERT(0);
}