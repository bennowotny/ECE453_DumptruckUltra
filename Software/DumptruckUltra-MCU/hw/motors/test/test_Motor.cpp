#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"
#include "motors/Motor.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::Motors::Motor;
    using Hardware::Motors::MotorDirection;
    using Hardware::Motors::MotorPinDefinition;

    Motor uut{{.forwardPin = P9_3, .backwardPin = P7_1}, MotorDirection::REVERSE};
    uut.enable();

    float speed{0.0};
    bool goingUp{true};
    constexpr float STEP{100.0 / 20.0 / 100.0};

    // Climb the available duty cycles
    // FOR TESTER: Check that we use the full duty cycle and switch pins between forwards and backwards operation
    // If the direction is backwards, then expect the motors to start driving backwards
    while (true) {
        uut.setPower(speed);
        cyhal_system_delay_ms(10);
        if (goingUp && speed < 99)
            speed += STEP;
        else if (!goingUp && speed > -99)
            speed -= STEP;
        else {
            cyhal_system_delay_ms(5000);
            goingUp = !goingUp;
        }
    }
}