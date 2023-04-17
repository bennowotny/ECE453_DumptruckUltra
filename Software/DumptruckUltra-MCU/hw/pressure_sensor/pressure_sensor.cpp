#include "pressure_sensor.hpp"
#include "cy_result.h"
#include "cyhal_adc.h"
#include "pressure_sensor/pressure_sensor.hpp"
#include <cstdio>

// using namespace std;

namespace Hardware {
namespace Pressure_Sensor {

Pressure::Pressure(cyhal_gpio_t pin) : adcHandle{},
                                       adc_chan_0_obj{} {
    // Initialize ADC
    auto rslt{cyhal_adc_init(&adcHandle, pin, nullptr)};
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // Initialize ADC channel, allocate channel 0 to "pin"
    cyhal_adc_channel_config_t channel_config = {.enabled = true, .enable_averaging = false, .min_acquisition_ns = 220};
    auto result{cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adcHandle, pin, CYHAL_ADC_VNEG, &channel_config)};
    CY_ASSERT(CY_RSLT_SUCCESS == result);
}

int32_t Pressure::read() {
    // Read ADC conversion result for corresponding ADC channel
    auto adc_out{cyhal_adc_read(&adc_chan_0_obj)};
    return adc_out;
}

void Pressure::free() {
    // Release ADC and channel objects when no longer needed
    cyhal_adc_channel_free(&adc_chan_0_obj);
    cyhal_adc_free(&adcHandle);
}

} // namespace Pressure_Sensor
} // namespace Hardware