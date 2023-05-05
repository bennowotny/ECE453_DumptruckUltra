#ifndef HW_PRESSURE_SENSOR_HPP_
#define HW_PRESSURE_SENSOR_HPP_

#include "cyhal_adc.h"
#include "cyhal_hw_types.h"
#include <cstdint>

namespace Hardware {
namespace PressureSensor {

class PressureSensor {
public:
    explicit PressureSensor(cyhal_gpio_t pin);
    auto read() -> int32_t;
    auto isPressed() -> bool;
    auto free() -> void;

private:
    cyhal_adc_t adcHandle;
    cyhal_adc_channel_t adc_chan_0_obj;
};

} // namespace PressureSensor
} // namespace Hardware

#endif