#pragma once

#include "FreeRTOS.h"
#include "hw/i2c/i2cBusManager.hpp"
#include "portmacro.h"
#include "queue.h"
#include <array>
#include <cstddef>

class IMU {
public:
    struct imuData_t {
        std::array<float, 3> accel_float; // X, Y, Z
        std::array<float, 3> gyro_float;  // X, Y, Z
        uint32_t timestamp;
    };

    IMU(const Hardware::I2C::I2CBusManager::i2cPin_t &);

    template <std::size_t N>
    auto getIMUData(std::array<imuData_t, N> dataBuf, TickType_t tickToWait) -> void;

private:
    static constexpr uint8_t IMU_ADDR{0x6A};

    Hardware::I2C::I2CBusManager i2cBus;
    // Queue of some sort. FreeRTOS Queue?
    QueueHandle_t imuQueue;
};