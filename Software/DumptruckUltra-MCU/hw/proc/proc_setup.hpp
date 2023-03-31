#ifndef _HW_PROC_PROC_SETUP_HPP
#define _HW_PROC_PROC_SETUP_HPP

#include "cyhal.h" // IWYU pragma: keep

namespace Hardware {
namespace Processor {

constexpr cyhal_gpio_t SERVO1_PIN{P0_0};

void setupProcessor();
} // namespace Processor
} // namespace Hardware

#endif