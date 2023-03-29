#include "Servo.hpp"
#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"

auto main() -> int {

    Hardware::Processor::setupProcessor();

    // Test Setup
    using Hardware::Servos::Servo;
    Servo testServo{P9_3};
    testServo.enable();

    float position{0.0};
    bool goingUp{true};
    constexpr float STEP{100.0 / 20.0 / 100.0};

    // Climb the available duty cycles
    // FOR TESTER: Check w/ scope that the lowest high time is 600us and the highest high tim is 2400 us
    while (true) {
        testServo.setPosition(position);
        cyhal_system_delay_ms(10);
        if (goingUp && position < 99)
            position += STEP;
        else if (!goingUp && position > 1)
            position -= STEP;
        else
            goingUp = !goingUp;
    }
}