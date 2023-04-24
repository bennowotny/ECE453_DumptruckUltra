#pragma once

#include "FreeRTOS.h"
#include "cy_syslib.h"
#include "cyhal_gpio.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "hw/i2c/i2cBusManager.hpp"
#include "portmacro.h"
#include "projdefs.h"
#include "queue.h"
#include <array>
#include <functional>
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
    explicit IMU(std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus,
                 std::function<void(const AccelerometerData &)> sendAccelData,
                 std::function<void(const GyroscopeData &)> sendGyroData);

private:
    TaskHandle_t imuTaskHandle;
    const std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus;
    const std::function<void(AccelerometerData &)> sendAccelData;
    const std::function<void(GyroscopeData &)> sendGyroData;

    auto imuTask() -> void;

    static constexpr UBaseType_t IMU_TASK_PRIORITY{tskIDLE_PRIORITY + 2};
    static constexpr cyhal_gpio_t IMU_INT_PIN{P9_2};
    static constexpr uint8_t IMU_ADDR{0x6A};

    // REGISTERS
    // Sensor control registers
    static constexpr uint8_t CTRL1_XL{0x10};
    static constexpr uint8_t CTRL2_G{0x11};
    static constexpr uint8_t CTRL9_XL{0x18};
    static constexpr uint8_t CTRL3_C{0x12};

    // FIFO control registers
    static constexpr uint8_t FIFO_CTRL1{0x0};
    static constexpr uint8_t FIFO_CTRL2{0x08};
    static constexpr uint8_t FIFO_CTRL3{0X09};
    static constexpr uint8_t FIFO_CTRL4{0x0A};

    // Interrupt control registers
    static constexpr uint8_t COUNTER_BDR_REG1{0x0B};
    static constexpr uint8_t COUNTER_BDR_REG2{0x0C};
    static constexpr uint8_t INT1_CTRL{0x0D};

    // Device address register
    static constexpr uint8_t WHO_AM_I{0x0F};

    // Sensor data registers
    /* THIS COMMENT BLOCK IS NOT ACCURATE!
    // Address of status register. Followed by register for temp sensor data,
    // gyroscope data, and accelerometer data.
    */
    // Starts at address of OUTX_L_G. Subsequent registers include rest of
    // gyroscope data and accelerometer data
    static constexpr uint8_t SENSOR_DATA_BEGIN{0x22};

    // FIFO data registers
    static constexpr uint8_t FIFO_DATA_OUT_TAG{0x78};
    static constexpr uint8_t FIFO_DATA_OUT_BEGIN{0x78};
    static constexpr uint8_t FIFO_STATUS1{0x3A};

    // Timestamp register
    static constexpr uint8_t TIMESTAMP_REGS{0x40};

    // CONTROL WORDS
    static constexpr uint8_t IMU_DEV_ID{0x6C};

    static constexpr uint8_t XL_CTRL{0x40}; // Accelerometer ODR and scale
    static constexpr uint8_t G_CTRL{0x4C};  // Gyroscope ODR and scale
    static constexpr uint8_t XL_DISABLE_I3C{0xE2};

    static constexpr uint8_t SET_BLK_DATA_UPDATE{0x40};
    static constexpr uint8_t SET_AUTO_INCREMENT{0x04}; // Autoincrement register address on multibyte read

    static constexpr uint8_t FIFO_MODE{0x01};
    static constexpr uint8_t CONT_MODE{0x06};

    static constexpr uint8_t CNT_BDR_TH{10}; // Threshold for BDR counter interrupt
    static constexpr uint8_t EN_BDR_INT{0x40};

    // APPLICATION CONSTANTS
    static constexpr uint8_t GYRO_DATA_TAG{0x01};
    static constexpr uint8_t ACCEL_DATA_TAG{0x02};

    static constexpr float TIMESTAMP_RES{0.025};
    static constexpr float GYRO_SCALE{70.0F / 1000.0F};
    static constexpr float ACCEL_SCALE{9.81F * 0.061F / 1000.0F};

    static constexpr uint8_t READ_INTERVAL_MS{10};
};
} // namespace IMU
} // namespace Hardware