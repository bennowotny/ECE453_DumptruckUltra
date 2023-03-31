/*
 * Servo.hpp
 *
 *  Created on: Mar 20, 2023
 *      Author: Ben Nowotny
 */

#ifndef HW_SERVOS_SERVO_HPP_
#define HW_SERVOS_SERVO_HPP_

#include "cyhal_hw_types.h"
#include <cstdint>

namespace Hardware {
namespace Servos {

class Servo {
public:
    explicit Servo(cyhal_gpio_t pin);
    void enable();
    void disable();
    void setPosition(float position);

private:
    cyhal_pwm_t pwmHandle;
    // A guess, based on servo properties
    // The servo requires 600us of high time at the low position and 2400us of high time in the high position
    // Does not say what frequency it needs
    static constexpr uint32_t SERVO_PWM_FREQUENCY_HZ{333};
    // Calculations for duty cycle boundaries made in ms
    static constexpr float SERVO_PWM_LOW_DUTY_CYCLE{0.6 / (1000.0 / SERVO_PWM_FREQUENCY_HZ) * 100};
    static constexpr float SERVO_PWM_HIGH_DUTY_CYCLE{2.4 / (1000.0 / SERVO_PWM_FREQUENCY_HZ) * 100};

    // Servo positions are floats on [0,100]
    static constexpr float SERVO_POSITION_LOWER_BOUND{0};
    static constexpr float SERVO_POSITION_UPPER_BOUND{100};
};

} // namespace Servos
} // namespace Hardware

#endif /* HW_SERVO_SERVO_HPP_ */
