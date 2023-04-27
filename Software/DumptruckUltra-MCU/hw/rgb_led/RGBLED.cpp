#include "RGBLED.hpp"
#include "cy_result.h"
#include "cyhal_pwm.h"
#include "rgb_led/RGBLED.hpp"

namespace Hardware {
namespace RGB_LED {

RGBLED::RGBLED(const RGBLayout &layout)
    : redPWMHandle{},
      greenPWMHandle{},
      bluePWMHandle{} {

    const auto redResult{cyhal_pwm_init(&redPWMHandle, layout.redPin, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == redResult);

    const auto greenResult{cyhal_pwm_init(&greenPWMHandle, layout.greenPin, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == greenResult);

    const auto blueResult{cyhal_pwm_init(&bluePWMHandle, layout.bluePin, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == blueResult);
}

void RGBLED::turnOn() {
    const auto redResult{cyhal_pwm_start(&redPWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == redResult);

    const auto greenResult{cyhal_pwm_start(&greenPWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == greenResult);

    const auto blueResult{cyhal_pwm_start(&bluePWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == blueResult);
}

void RGBLED::turnOff() {
    const auto redResult{cyhal_pwm_stop(&redPWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == redResult);

    const auto greenResult{cyhal_pwm_stop(&greenPWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == greenResult);

    const auto blueResult{cyhal_pwm_stop(&bluePWMHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == blueResult);
}

void RGBLED::setColor(const Color &color) {
    const float redPWMDutyCycle{(static_cast<float>(color.red) / MAX_COLOR_VALUE) * MAX_PWM_DUTY_CYCLE};
    const auto redResult{cyhal_pwm_set_duty_cycle(&redPWMHandle, redPWMDutyCycle, RGB_LED_FREQUENCY)};
    CY_ASSERT(CY_RSLT_SUCCESS == redResult);

    const float greenPWMDutyCycle{(static_cast<float>(color.green) / MAX_COLOR_VALUE) * MAX_PWM_DUTY_CYCLE};
    const auto greenResult{cyhal_pwm_set_duty_cycle(&greenPWMHandle, greenPWMDutyCycle, RGB_LED_FREQUENCY)};
    CY_ASSERT(CY_RSLT_SUCCESS == greenResult);

    const float bluePWMDutyCycle{(static_cast<float>(color.blue) / MAX_COLOR_VALUE) * MAX_PWM_DUTY_CYCLE};
    const auto blueResult{cyhal_pwm_set_duty_cycle(&bluePWMHandle, bluePWMDutyCycle, RGB_LED_FREQUENCY)};
    CY_ASSERT(CY_RSLT_SUCCESS == blueResult);
}

} // namespace RGB_LED
} // namespace Hardware