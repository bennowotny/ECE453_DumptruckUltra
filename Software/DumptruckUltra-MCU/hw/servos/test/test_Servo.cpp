#include "Servo.hpp"
#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"

auto main() -> int {

    Hardware::Processor::setupProcessor();

    // Test Setup
    using Hardware::Servos::Servo;
    Servo testServo{Hardware::Processor::SERVO1_PWM};
    testServo.enable();

    float position{0.0};
    bool goingUp{true};
    constexpr float STEP{100.0 / 20.0 / 100.0};

    // Climb the available duty cycles, pausing at the max and min
    // FOR TESTER: Check w/ scope that the lowest LOW time is 600us and the highest high tim is 2400 us
    // Or high time, if we're analyzing after the hardware inversion
    // While the position is changing, there is a 'break' every 20ms as we re-configure the duty cycle.  This is normal
    while (true) {
        testServo.setPosition(position);
        cyhal_system_delay_ms(10);
        if (goingUp && position < 99)
            position += STEP;
        else if (!goingUp && position > 1)
            position -= STEP;
        else {
            cyhal_system_delay_ms(5000);
            goingUp = !goingUp;
        }
    }
}