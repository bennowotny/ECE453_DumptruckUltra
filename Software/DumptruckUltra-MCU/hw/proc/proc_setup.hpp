#ifndef _HW_PROC_PROC_SETUP_HPP
#define _HW_PROC_PROC_SETUP_HPP

#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "cycfg_pins.h"
#include "cyhal.h" // IWYU pragma: keep
#include "task.h"  // IWYU pragma: keep
#include <string>

namespace Hardware {
<<<<<<< HEAD
}

namespace Processor {
    constexpr cyhal_gpio_t MCU_RX_PI_TX(P5_0);
    constexpr cyhal_gpio_t MCU_TX_PI_RX(P5_1);
    constexpr cyhal_gpio_t MI_FORWARD(P5_2);
    constexpr cyhal_gpio_t Servo1_PWM(P5_3);
    constexpr cyhal_gpio_t M1_Backward(P5_4);
    constexpr cyhal_gpio_t Servo2_PWM(P5_5);
    constexpr cyhal_gpio_t M2_Forward(P5_6);

    constexpr cyhal_gpio_t M2_Backward(P6_2);
    constexpr cyhal_gpio_t Servo3_PWM(P6_3);
    constexpr cyhal_gpio_t MCU_RX_FTDI_TX(P6_4);
    constexpr cyhal_gpio_t MCU_TX_FTDI_RX(P6_5);
    constexpr cyhal_gpio_t SWDIO(P6_6);
    constexpr cyhal_gpio_t SWDCLK(P6_7);

    constexpr cyhal_gpio_t USER_LED(P7_1);
    constexpr cyhal_gpio_t PB_IN (P7_2);

    constexpr cyhal_gpio_t I2C_SCL (P9_0);
    constexpr cyhal_gpio_t I2C_SDA (P9_1);
    constexpr cyhal_gpio_t IMU_INT (P9_2);
    constexpr cyhal_gpio_t DIST_INT (P9_3);

    constexpr cyhal_gpio_t PressureSensorADC (P10_0);
    constexpr cyhal_gpio_t Servo4_PWM (P10_1);
    constexpr cyhal_gpio_t Servo5_PWM (P10_3);
    constexpr cyhal_gpio_t Servo6_PWM (P10_5);
    constexpr cyhal_gpio_t Servo7_PWM (P12_7);


void setupProcessor();
}

=======

namespace Processor {
constexpr cyhal_gpio_t MCU_RX_PI_TX{P5_0};
constexpr cyhal_gpio_t MCU_TX_PI_RX{P5_1};

constexpr cyhal_gpio_t MI_FORWARD{P5_2};
constexpr cyhal_gpio_t M1_BACKWARD{P5_4};
constexpr cyhal_gpio_t M2_FORWARD{P5_6};
constexpr cyhal_gpio_t M2_BACKWARD{P6_2};

constexpr cyhal_gpio_t SERVO1_PWM{P5_3};
constexpr cyhal_gpio_t SERVO2_PWM{P5_5};
constexpr cyhal_gpio_t SERVO3_PWM{P6_3};
constexpr cyhal_gpio_t SERVO4_PWM{P10_1};
constexpr cyhal_gpio_t SERVO5_PWM{P10_3};
constexpr cyhal_gpio_t SERVO6_PWM{P10_5};
constexpr cyhal_gpio_t SERVO7_PWM{P12_7};

constexpr cyhal_gpio_t MCU_RX_FTDI_TX{P6_4};
constexpr cyhal_gpio_t MCU_TX_FTDI_RX{P6_5};

// SWDIO already defined
constexpr cyhal_gpio_t SWDCLK{P6_7};

constexpr cyhal_gpio_t USER_LED{P7_1};
constexpr cyhal_gpio_t USER_RGB_RED{P9_4};
constexpr cyhal_gpio_t USER_RGB_GREEN{P9_5};
constexpr cyhal_gpio_t USER_RGB_BLUE{P9_6};
constexpr cyhal_gpio_t PB_IN{P7_2};

constexpr cyhal_gpio_t I2C_SCL{P9_0};
constexpr cyhal_gpio_t I2C_SDA{P9_1};

constexpr cyhal_gpio_t IMU_INT{P9_2};

constexpr cyhal_gpio_t DIST_INT{P9_3};

constexpr cyhal_gpio_t PRESSURE_SENSOR_ADC{P10_0};

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
>>>>>>> cc6e3ab7698932bb9a79b60c204213e87f9a78bc
} // namespace Hardware


#endif