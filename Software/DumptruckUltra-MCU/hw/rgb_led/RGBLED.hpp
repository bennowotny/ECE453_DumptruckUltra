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

struct Color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

namespace PredefinedColors {
constexpr Color RED{.red = 255, .green = 0, .blue = 0};
constexpr Color GREEN{.red = 0, .green = 255, .blue = 0};
constexpr Color BLUE{.red = 0, .green = 0, .blue = 255};
constexpr Color YELLOW{.red = 255, .green = 255, .blue = 0};
constexpr Color MAGENTA{.red = 255, .green = 0, .blue = 255};
constexpr Color CYAN{.red = 0, .green = 255, .blue = 255};
constexpr Color WHITE{.red = 255, .green = 255, .blue = 255};
constexpr Color BLACK{.red = 0, .green = 0, .blue = 0};
} // namespace PredefinedColors

class RGBLED {
public:
    explicit RGBLED(const RGBLayout &layout);

    void turnOn();

    void turnOff();

    void setColor(const Color &color);

private:
    cyhal_pwm_t redPWMHandle;
    cyhal_pwm_t greenPWMHandle;
    cyhal_pwm_t bluePWMHandle;

    static constexpr float MAX_COLOR_VALUE{255};
    static constexpr float MAX_PWM_DUTY_CYCLE{100};
    static constexpr uint32_t RGB_LED_FREQUENCY{500};
};
} // namespace RGB_LED
} // namespace Hardware

#endif