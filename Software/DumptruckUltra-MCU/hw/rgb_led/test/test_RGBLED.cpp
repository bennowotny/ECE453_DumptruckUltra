#include "proc_setup.hpp"
#include "rgb_led/RGBLED.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::RGB_LED::Color;
    using Hardware::RGB_LED::RGBLED;

    RGBLED uut{
        {.redPin = Hardware::Processor::USER_RGB_RED,
         .greenPin = Hardware::Processor::USER_RGB_GREEN,
         .bluePin = Hardware::Processor::USER_RGB_BLUE}};

    // FOR TESTER: Change the color here (predefined or otherwise) to change the color for the test.
    uut.setColor(Hardware::RGB_LED::PredefinedColors::RED);
    bool isOn{false};

    while (true) {
        if (isOn) {
            uut.turnOff();
        } else {
            uut.turnOn();
        }
        cyhal_system_delay_ms(500);
    }
}