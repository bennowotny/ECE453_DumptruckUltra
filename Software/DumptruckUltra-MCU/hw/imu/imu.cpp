#include "hw/imu/imu.hpp"
#include "FreeRTOS.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"

IMU::IMU(const Hardware::I2C::I2CBusManager::i2cPin_t &i2cPins)
    : i2cBus(i2cPins) {
    // Write CTRL1_XL AND CTRL2_G to set Output Data Rate (ODR)
    // Write CTRL6_C?
    pdMS_TO_TICKS(5);
}