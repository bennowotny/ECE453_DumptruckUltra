/*
 * Servo.cpp
 *
 *  Created on: Mar 20, 2023
 *      Author: Ben Nowotny
 */

#include "Servo.hpp"

namespace Hardware{
namespace Servos{

Servo::Servo(cyhal_gpio_pin pin){
	auto result{cyhal_pwm_init(&pwmHandle, pin, nullptr)};
	CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::enable(){
	auto result{cyhal_pwm_start(&pwmHandle)};
	CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::disable(){
	auto result{cyhal_pwm_stop(&pwmHandle)};
	CY_ASSERT(CY_RSLT_SUCCESS == result);
}

void Servo::setPosition(float position){

	const float positionScaledFromInput{(position - SERVO_POSITION_LOWER_BOUND) / (SERVO_POSITION_UPPER_BOUND - SERVO_POSITION_LOWER_BOUND)};
	const float positionScaledToOutput{(positionScaledFromInput * (SERVO_PWM_HIGH_DUTY_CYCLE - SERVO_PWM_LOW_DUTY_CYCLE)) + SERVO_PWM_LOW_DUTY_CYCLE};

	auto result{cyhal_pwm_set_duty_cycle(&pwmHandle, positionScaledToOutput, SERVO_PWM_FREQUENCY_HZ)};
	CY_ASSERT(CY_RSLT_SUCCESS == result);
}

}
}


