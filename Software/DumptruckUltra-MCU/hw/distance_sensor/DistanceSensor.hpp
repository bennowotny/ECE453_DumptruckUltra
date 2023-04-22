#ifndef __HW_DISTANCE_SENSOR_DISTANCESENSOR_HPP
#define __HW_DISTANCE_SENSOR_DISTANCESENSOR_HPP

#include "i2cBusManager.hpp"
#include <memory>

namespace Hardware {
namespace DistanceSensor {

class DistanceSensor {
public:
    explicit DistanceSensor(std::shared_ptr<I2C::I2CBusManager> busManager, uint16_t deviceAddress);
    auto getDistanceMeters() const -> float;
    // auto getDistanceMetersPoll() const -> float;

    static auto getI2CBusManager() -> I2C::I2CBusManager &; // Required for API usage

private:
    static std::shared_ptr<I2C::I2CBusManager> busManager; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables); required to interoperate with C callbacks
    const uint16_t deviceAddress;

    static constexpr uint16_t DISTANCE_SENSOR_LONG_RANGE_MODE{2};
    static constexpr uint16_t DISTANCE_TIMING_BUDGET_MS{100};
    static constexpr uint16_t DISTANCE_INTER_MEASUREMENT_PERIOD_MS{100};
    static constexpr float MM_PER_METER{1000};
};

} // namespace DistanceSensor
} // namespace Hardware

#endif