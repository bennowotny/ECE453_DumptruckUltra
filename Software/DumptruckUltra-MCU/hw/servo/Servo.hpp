/*
 * Servo.hpp
 *
 *  Created on: Mar 20, 2023
 *      Author: Ben Nowotny
 */

#ifndef HW_SERVO_SERVO_HPP_
#define HW_SERVO_SERVO_HPP_

#include <cstdint>
#include "cyhal.h"
#include "cybsp.h"

namespace Hardware{
namespace Servos{

	using cyhal_gpio_pin=decltype(P0_0);

	class Servo{
	public:
		explicit Servo(cyhal_gpio_pin pin);
		void enable();
		void disable();
		void setPosition(float position);

	private:
		cyhal_pwm_t pwmHandle;
		static constexpr uint32_t SERVO_PWM_FREQUENCY_HZ{1000};
	};

}
}

#endif /* HW_SERVO_SERVO_HPP_ */
