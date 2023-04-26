#include "DistanceSensor.hpp"

#include "VL53L1X_api.h"
#include "cyhal_system.h"
#include "i2cBusManager.hpp"
#include <utility>

namespace Hardware {
namespace DistanceSensor {

// Req'd so that, when linking C functions, the linker looks up the right name.
extern "C" {
auto VL53L1X_BootState(uint16_t, uint8_t *) -> int;
auto VL53L1X_SensorInit(uint16_t) -> int;
auto VL53L1X_SetDistanceMode(uint16_t, uint16_t) -> int;
auto VL53L1X_SetTimingBudgetInMs(uint16_t, uint16_t) -> int;
auto VL53L1X_SetInterMeasurementInMs(uint16_t, uint16_t) -> int;
auto VL53L1X_StartRanging(uint16_t) -> VL53L1X_ERROR;
auto VL53L1X_ClearInterrupt(uint16_t) -> int;
auto VL53L1X_CheckForDataReady(uint16_t, uint8_t *) -> int;
auto VL53L1X_GetDistance(uint16_t, uint16_t *) -> VL53L1X_ERROR;
}

std::shared_ptr<I2C::I2CBusManager> DistanceSensor::busManager{}; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables); required to interoperate with C callbacks

DistanceSensor::DistanceSensor(std::shared_ptr<I2C::I2CBusManager> busManager, uint16_t deviceAddress)
    : deviceAddress{deviceAddress} {
    DistanceSensor::busManager = std::move(busManager);

    uint8_t sensorState{0};
    while (sensorState == 0) {
        VL53L1X_BootState(this->deviceAddress, &sensorState);
        cyhal_system_delay_ms(1); // Wait for boot
    }

    VL53L1X_SensorInit(this->deviceAddress);
    VL53L1X_SetDistanceMode(this->deviceAddress, DISTANCE_SENSOR_LONG_RANGE_MODE);
    VL53L1X_SetTimingBudgetInMs(this->deviceAddress, DISTANCE_TIMING_BUDGET_MS);
    VL53L1X_SetInterMeasurementInMs(this->deviceAddress, DISTANCE_INTER_MEASUREMENT_PERIOD_MS);
    VL53L1X_StartRanging(this->deviceAddress);
}

/**
 * Note: This implementation does not continually poll the sensor,
 * only gets a single "on-demand" reading from the sensor
 */
auto DistanceSensor::getDistanceMeters() const -> float {
    VL53L1X_ClearInterrupt(deviceAddress);
    uint8_t dataReady{0};
    while (dataReady == 0) {
        VL53L1X_CheckForDataReady(deviceAddress, &dataReady);
    }
    uint16_t distance_mm{0};
    VL53L1X_GetDistance(deviceAddress, &distance_mm);
    return static_cast<float>(distance_mm) / MM_PER_METER;
}

// auto DistanceSensor::getDistanceMetersPoll() const -> float {
//     VL53L1X_ClearInterrupt(deviceAddress);
//     uint8_t dataReady{0};
//     while (dataReady == 0) {
//         VL53L1X_CheckForDataReady(deviceAddress, &dataReady);
//     }
//     return static_cast<float>(distance_mm) / MM_PER_METER;

// }

auto DistanceSensor::getI2CBusManager() -> I2C::I2CBusManager & {
    return *DistanceSensor::busManager;
}

} // namespace DistanceSensor
} // namespace Hardware