#ifndef _HW_PROC_PROC_SETUP_HPP
#define _HW_PROC_PROC_SETUP_HPP

#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "cyhal.h" // IWYU pragma: keep
#include "task.h"  // IWYU pragma: keep

namespace Hardware {
namespace Processor {

constexpr cyhal_gpio_t SERVO1_PIN{P0_0};

void setupProcessor();

class FreeRTOSBlinky {
public:
    explicit FreeRTOSBlinky(cyhal_gpio_t blinkyPin);

private:
    [[noreturn]] static void ledTask(FreeRTOSBlinky *obj);

    const cyhal_gpio_t blinkyPin;

    static constexpr auto BLINKY_TASK_NAME{"Blinky"};
    static constexpr uint32_t BLINKY_STACK_SIZE{configMINIMAL_STACK_SIZE};
    static constexpr uint32_t BLINKY_PRIORITY{tskIDLE_PRIORITY + 1};
};

} // namespace Processor
} // namespace Hardware

#endif