/*
 * Servo.cpp
 *
 *  Created on: Mar 20, 2023
 *      Author: Ben Nowotny
 */

#include "Servo.hpp"
#include "cyhal_pwm.h"
#include "cyhal_system.h"
#include <cmath>

namespace Hardware {
namespace Servos {

Servo::Servo(cyhal_gpio_t pin) : pwmHandle{} {
    // Invert so that we can use the hardware inversion to produce a normal signal
    auto result{cyhal_pwm_init_adv(&pwmHandle, pin, NC, CYHAL_PWM_RIGHT_ALIGN, true, 0U, true, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::enable() {
    auto result{cyhal_pwm_start(&pwmHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::disable() {
    auto result{cyhal_pwm_stop(&pwmHandle)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::setPosition(float position) {

    const float positionScaledFromInput{(position - SERVO_POSITION_LOWER_BOUND) / (SERVO_POSITION_UPPER_BOUND - SERVO_POSITION_LOWER_BOUND)};
    const float positionScaledToOutput{(positionScaledFromInput * (SERVO_PWM_HIGH_DUTY_CYCLE - SERVO_PWM_LOW_DUTY_CYCLE)) + SERVO_PWM_LOW_DUTY_CYCLE};

    auto result{cyhal_pwm_set_duty_cycle(&pwmHandle, positionScaledToOutput, SERVO_PWM_FREQUENCY_HZ)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::sweep(float start, float end, SweepConfiguration sweepParams) {

    auto currentPosition{start};
    //const auto step{(end - start) / static_cast<float>(sweepParams.stepCount)};
    const auto step{1};

    if(end>start){
        while(currentPosition<end){
            setPosition(currentPosition);
            cyhal_system_delay_ms(sweepParams.stepDuration_ms);
            currentPosition += step;
        }
    }else{
        while(currentPosition>end){
            setPosition(currentPosition);
            cyhal_system_delay_ms(sweepParams.stepDuration_ms);
            currentPosition -= step;
        }
    }

    /*
    while (std::abs(end - currentPosition) > step) {
        setPosition(currentPosition);
        cyhal_system_delay_ms(sweepParams.stepDuration_ms);
        currentPosition += step;
    }
    */
    setPosition(end);
}

} // namespace Servos
} // namespace Hardware
