#ifndef HW_MOTORS_MOTOR_HPP_
#define HW_MOTORS_MOTOR_HPP_

#include "cyhal_hw_types.h"
#include "hw/proc/proc_setup.hpp"

namespace Hardware {
namespace Motors {

using Hardware::Processor::cyhal_gpio_pin;

struct MotorPinDefinition {
    cyhal_gpio_pin forwardPin;
    cyhal_gpio_pin backwardPin;
};

class Motor {
public:
    Motor(MotorPinDefinition motorPins);
    void enable();
    void disable();
    void setPower(float power);

private:
    cyhal_pwm_t forwardPWMHandle;
    cyhal_pwm_t backwardPWMHandle;

    // A guess.
    // There shouldn't be any protocol specific requirements for the frequency of an H-bridge (maybe upper limit for max switching frequency).
    // Just controls how much resolution we have in the power delivered.
    static constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ{500};
};

} // namespace Motors
} // namespace Hardware

#endif