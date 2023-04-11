#pragma once

#include "FreeRTOS.h"
#include "hw/i2c/i2cBusManager.hpp"
#include "portmacro.h"
#include "queue.h"
#include <array>
#include <cstddef>
#include <memory>

namespace Hardware {
namespace IMU {

struct AccelerometerData {
    float Ax;
    float Ay;
    float Az;
    float Ats;
};

struct GyroscopeData {
    float Gx;
    float Gy;
    float Gz;
    float Gts;
};

class IMU {
public:
    explicit IMU(std::shared_ptr<Hardware::I2C::I2CBusManager>);

private:
    static constexpr uint8_t IMU_ADDR{0x6A};
    const std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus;
};
} // namespace IMU
} // namespace Hardware