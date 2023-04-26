#include "pressure_sensor/pressure_sensor.hpp"
#include "cy_result.h"
#include "cyhal_adc.h"
#include <cstdio>

// using namespace std;

namespace Hardware {
namespace PressureSensor {

PressureSensor::PressureSensor(cyhal_gpio_t pin) : adcHandle{},
                                                   adc_chan_0_obj{} {
    // Initialize ADC
    auto rslt{cyhal_adc_init(&adcHandle, pin, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // Initialize ADC channel, allocate channel 0 to "pin"
    cyhal_adc_channel_config_t channel_config = {.enabled = true, .enable_averaging = false, .min_acquisition_ns = 220};
    auto result{cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adcHandle, pin, CYHAL_ADC_VNEG, &channel_config)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

int32_t PressureSensor::read() {
    // Read ADC conversion result for corresponding ADC channel
    auto adc_out{cyhal_adc_read(&adc_chan_0_obj)};
    return adc_out;
}

bool PressureSensor::isPressed() {
    // Placeholder value until tested
    return read() > 200;
}

} // namespace PressureSensor
} // namespace Hardware