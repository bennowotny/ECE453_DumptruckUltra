#ifndef HW_RGB_LED_RGBLED_HPP
#define HW_RGB_LED_RGBLED_HPP

#include "cyhal.h" // IWYU pragma: keep

namespace Hardware {
namespace RGB_LED {

struct RGBLayout {
    const cyhal_gpio_t redPin;
    const cyhal_gpio_t greenPin;
    const cyhal_gpio_t bluePin;
};



class RGBLED {
public:
    explicit RGBLED(const RGBLayout &layout);

private:
    cyhal_pwm_t redPWMHandle;
    cyhal_pwm_t greenPWMHandle;
    cyhal_pwm_t bluePWMHandle;
};
} // namespace RGB_LED
} // namespace Hardware

#endif