#include "hw/imu/imu.hpp"
#include "FreeRTOS.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
#include <memory>

namespace Hardware {
namespace IMU {
IMU::IMU(std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus) : i2cBus{std::move(i2cBus)} {
    // Write CTRL1_XL AND CTRL2_G to set Output Data Rate (ODR)
    // Write CTRL6_C?

    // Set up interrupt pin

    // Initialize sensors

    // Check device ID
}
} // namespace IMU
} // namespace Hardware