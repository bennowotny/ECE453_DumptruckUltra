#ifndef _HW_PROC_PROC_SETUP_HPP
#define _HW_PROC_PROC_SETUP_HPP

#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "cyhal.h" // IWYU pragma: keep
#include "task.h"  // IWYU pragma: keep
#include <string>

namespace Hardware {
namespace Processor {

constexpr auto MCU_RX_PI_TX{P5_0};
constexpr auto MCU_TX_PI_RX{P5_1};
constexpr auto M1_FORWARD{P5_2};
constexpr auto M1_BACKWARD{P5_4};
constexpr auto M2_FORWARD{P5_6};

constexpr auto M2_BACKWARD{P6_2};
constexpr auto MCU_RX_FTDI_TX{P6_4};
constexpr auto MCU_TX_FTDI_RX{P6_5};
// SWDIO already defined
constexpr auto SWDCLK{P6_7};

constexpr auto USER_LED{P7_1};
constexpr auto PB_IN{P7_2};

constexpr auto I2C_SCL{P9_0};
constexpr auto I2C_SDA{P9_1};
constexpr auto IMU_INT{P9_2};
constexpr auto DIST_INT{P9_3};

constexpr auto PRESSURE_SENSOR_ADC{P10_0};

constexpr auto SERVO1_PWM{P5_3};
constexpr auto SERVO2_PWM{P5_5};
constexpr auto SERVO3_PWM{P6_3};
constexpr auto SERVO4_PWM{P10_1};
constexpr auto SERVO5_PWM{P10_3};
constexpr auto SERVO6_PWM{P10_5};
constexpr auto SERVO7_PWM{P12_7};

constexpr auto USER_RGB_RED{P9_4};
constexpr auto USER_RGB_GREEN{P9_5};
constexpr auto USER_RGB_BLUE{P9_6};

void setupProcessor();

class FreeRTOSBlinky {
public:
    explicit FreeRTOSBlinky(cyhal_gpio_t blinkyPin, uint32_t delayMs = 0, const char *taskName = BLINKY_TASK_NAME);

private:
    [[noreturn]] void ledTask();

    const cyhal_gpio_t blinkyPin;
    const uint32_t delayMs;

    static char const *const BLINKY_TASK_NAME;
    static constexpr uint32_t BLINKY_STACK_SIZE{configMINIMAL_STACK_SIZE};
    static constexpr uint32_t BLINKY_PRIORITY{tskIDLE_PRIORITY + 1};
};

} // namespace Processor
} // namespace Hardware

#endif