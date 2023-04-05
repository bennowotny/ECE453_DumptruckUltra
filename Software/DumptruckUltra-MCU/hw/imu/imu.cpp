#include "hw/imu/imu.hpp"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2c/i2cBusManager.hpp"

IMU(cyhal_gpio_t sda, cyhal_gpio_t scl) : i2cBus(sda, scl) {
    // Write CTRL1_XL AND CTRL2_G to set Output Data Rate (ODR)
    // Write CTRL6_C?
}