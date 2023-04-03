#pragma once

#include "cyhal_psoc6_01_43_smt.h"
#include "i2c/i2cBusManager.hpp"

class IMU {
public:
    IMU();

private:
    I2CBusManager i2cBus;
    // Queue of some sort. FreeRTOS Queue?
}