#pragma once

#include "FreeRTOS.h"
#include "cy_scb_i2c.h"
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
                 std::function<void(const GyroscopeData &)> sendGyroData,
                 std::function<void(const char *)> printStr);

private:
    TaskHandle_t imuTaskHandle;
    const std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus;
    const std::function<void(AccelerometerData &)> sendAccelData;
    const std::function<void(GyroscopeData &)> sendGyroData;
    // const std::function<void(uint8_t)> printRaw;
    const std::function<void(const char *)> printStr;
    float avg_xl_x_off;
    float avg_xl_y_off;
    float avg_xl_z_off;
    float avg_g_x_off;
    float avg_g_y_off;
    float avg_g_z_off;
    std::array<float, 3> avgXlX;
    std::array<float, 3> avgXlY;
    std::array<float, 3> avgGZ;
    uint8_t bufInd;

    auto imuTask() -> void;
    auto calibrate() -> void;
    auto addToRingBuffers(float newXlX, float newXlY, float newGZ) -> void;
    auto getCurrReadings() -> const std::array<float, 3>;

    static constexpr UBaseType_t IMU_TASK_PRIORITY{tskIDLE_PRIORITY + 3};
    static constexpr cyhal_gpio_t IMU_INT_PIN{P9_2};
    static constexpr uint8_t IMU_ADDR{0x6A};

    // REGISTERS
    // Sensor control registers
    static constexpr uint8_t CTRL1_XL{0x10};
    static constexpr uint8_t CTRL2_G{0x11};
    static constexpr uint8_t CTRL8_XL{0x17};
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
    static constexpr uint8_t SENSOR_DATA_STATUS{0x1E};

    // FIFO data registers
    static constexpr uint8_t FIFO_DATA_OUT_TAG{0x78};
    static constexpr uint8_t FIFO_DATA_OUT_BEGIN{0x78};
    static constexpr uint8_t FIFO_STATUS1{0x3A};

    // Timestamp register
    static constexpr uint8_t TIMESTAMP_REGS{0x40};

    // CONTROL WORDS
    static constexpr uint8_t IMU_DEV_ID{0x6C};

    // static constexpr uint8_t XL_CTRL{0x40}; // Accelerometer ODR and scale
    // static constexpr uint8_t XL_CTRL{0x44};        // Accelerometer ODR and scale
    // static constexpr uint8_t XL_CTRL{0x46}; // Accelerometer ODR, scale, second stage filter enable

    static constexpr uint8_t XL_ODR_104{0x40};
    static constexpr uint8_t XL_ODR_208{0x50};
    static constexpr uint8_t XL_SET_LPF2_EN{0x02};
    static constexpr uint8_t XL_FS_SCALE_2G{0x00};
    static constexpr uint8_t XL_FS_SCALE_4G{0x10};
    static constexpr uint8_t XL_FS_SCALE_8G{0x11};
    static constexpr uint8_t XL_FS_SCALE_16G{0x01};

    static constexpr uint8_t XL_SET_LPF2_ODR_OVER_10{0x20};
    static constexpr uint8_t XL_SET_LPF2_ODR_OVER_45{0x60};
    static constexpr uint8_t XL_SET_LPF2_ODR_OVER_100{0x80};

    static constexpr uint8_t G_CTRL{0x4C}; // Gyroscope ODR and scale
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
    // static constexpr float ACCEL_SCALE{9.81F * 0.488F / 1000.0F};

    static constexpr uint8_t READ_INTERVAL_MS{10};
    static constexpr uint16_t NUM_CALIBRATION_SAMPLES{500};
    static constexpr uint8_t NUM_CALIBRATION_PASSES{3};
    static constexpr uint8_t CIRC_BUF_LEN{5}; // Not used
};
} // namespace IMU
} // namespace Hardware