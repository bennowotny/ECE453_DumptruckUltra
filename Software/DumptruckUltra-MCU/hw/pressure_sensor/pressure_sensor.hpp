

#ifndef HW_PRESSURE_SENSOR_HPP_
#define HW_PRESSURE_SENSOR_HPP_

#include "cyhal_adc.h"
#include "cyhal_hw_types.h"
#include <cstdint>

namespace Hardware {
namespace Pressure_Sensor {

class Pressure {
public:
    explicit Pressure(cyhal_gpio_t pin);
    int32_t read();
    void free();

private:
    cyhal_adc_t adcHandle;
    cyhal_adc_channel_t adc_chan_0_obj;
};

} // namespace Pressure_Sensor
} // namespace Hardware

#endif