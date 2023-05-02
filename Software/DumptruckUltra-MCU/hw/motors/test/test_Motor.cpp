#include "cyhal_system.h"
#include "hw/proc/proc_setup.hpp"
#include "motors/Motor.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::Motors::Motor;
    using Hardware::Motors::MotorDirection;
    using Hardware::Motors::MotorPinDefinition;

    Motor uut{{.forwardPin = Hardware::Processor::M1_FORWARD, .backwardPin = Hardware::Processor::M1_BACKWARD}, MotorDirection::FORWARD};

    uut.setPower(0.5);
    uut.enable();

    Motor uut2{{.forwardPin = Hardware::Processor::M2_FORWARD, .backwardPin = Hardware::Processor::M2_BACKWARD}, MotorDirection::FORWARD};
    uut2.enable();
    uut2.setPower(0.25);

    while (true)
        ;
}