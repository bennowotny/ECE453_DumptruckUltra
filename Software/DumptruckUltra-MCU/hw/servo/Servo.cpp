/*
 * Servo.cpp
 *
 *  Created on: Mar 20, 2023
 *      Author: Ben Nowotny
 */

#include "Servo.hpp"

namespace Hardware::Servo{

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

void Servo::setPosition(uint8_t position){
	// FIXME
}

}


