#include "Motor.hpp"
#include "cy_result.h"
#include "cyhal_pwm.h"

namespace Hardware {
namespace Motors {
Motor::Motor(MotorPinDefinition motorPins, MotorDirection positiveDirection)
    : forwardPWMHandle{},
      backwardPWMHandle{},
      currentPower{0},
      positiveDirection{positiveDirection} {
    const auto resultForward{cyhal_pwm_init(&forwardPWMHandle, motorPins.forwardPin, nullptr)};
    CY_ASSERT(resultForward == CY_RSLT_SUCCESS);

    const auto resultBackward{cyhal_pwm_init(&backwardPWMHandle, motorPins.backwardPin, nullptr)};
    CY_ASSERT(resultBackward == CY_RSLT_SUCCESS);
}

void Motor::enable() {
    const auto resultForward{cyhal_pwm_start(&forwardPWMHandle)};
    CY_ASSERT(resultForward == CY_RSLT_SUCCESS);

    const auto resultBackward{cyhal_pwm_start(&backwardPWMHandle)};
    CY_ASSERT(resultBackward == CY_RSLT_SUCCESS);
}

void Motor::disable() {
    const auto resultForward{cyhal_pwm_stop(&forwardPWMHandle)};
    CY_ASSERT(resultForward == CY_RSLT_SUCCESS);

    const auto resultBackward{cyhal_pwm_stop(&backwardPWMHandle)};
    CY_ASSERT(resultBackward == CY_RSLT_SUCCESS);
}

void Motor::setPower(float power) {

    currentPower = power;

    if (positiveDirection == MotorDirection::REVERSE)
        power = -power;

    if (power >= 0) {
        cyhal_pwm_set_duty_cycle(&forwardPWMHandle, power, MOTOR_PWM_FREQUENCY_HZ);
        cyhal_pwm_set_duty_cycle(&backwardPWMHandle, 0, MOTOR_PWM_FREQUENCY_HZ);
    } else {
        cyhal_pwm_set_duty_cycle(&forwardPWMHandle, 0, MOTOR_PWM_FREQUENCY_HZ);
        cyhal_pwm_set_duty_cycle(&backwardPWMHandle, -power, MOTOR_PWM_FREQUENCY_HZ);
    }
}

auto Motor::getPower() const -> float {
    return currentPower;
}

} // namespace Motors
} // namespace Hardware