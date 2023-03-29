#include "hw/proc/proc_setup.hpp"
#include "motors/Motor.hpp"

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::Motors::Motor;
    using Hardware::Motors::MotorPinDefinition;

    Motor uut{{.forwardPin = P9_3, .backwardPin = P0_0}}; // TODO: Find pin

    while (true) {
    }
}