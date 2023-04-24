#include "hw/imu/imu.hpp"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cy_result.h"
#include "cy_utils.h"
#include "cyhal_gpio.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include <array>
#include <functional>
#include <memory>
#include <utility>

namespace Hardware {
namespace IMU {
// https://github.com/stm32duino/LSM6DSOX
IMU::IMU(std::shared_ptr<Hardware::I2C::I2CBusManager> i2cBus,
         std::function<void(const AccelerometerData &)> sendAccelData,
         std::function<void(const GyroscopeData &)> sendGyroData)
    : i2cBus{std::move(i2cBus)},
      sendAccelData{std::move(sendAccelData)},
      sendGyroData{std::move(sendGyroData)} {
    // TODO: take address as parameter

    // Query WHO_AM_I
    std::array<uint8_t, 1> dataToRec{};
    this->i2cBus->i2cRead1ByteReg(IMU_ADDR, WHO_AM_I, dataToRec);
    CY_ASSERT(dataToRec[0] == IMU_DEV_ID);

    // Disable I3C
    std::array<uint8_t, 1> dataToSend;
    dataToSend[0] = XL_DISABLE_I3C;
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, CTRL9_XL, dataToSend);

    // Enable block data update && autoincrement on read/write
    dataToSend[0] = SET_BLK_DATA_UPDATE | SET_AUTO_INCREMENT;
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, CTRL3_C, dataToSend);

    // Set XL and G scale
    // Enable XL and G w/ ODR
    dataToSend[0] = XL_CTRL;
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, CTRL1_XL, dataToSend);

    dataToSend[0] = G_CTRL;
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, CTRL2_G, dataToSend);

    // Configure BYPASS FIFO (clear data)
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, FIFO_CTRL4, {0x00});

    // FIXME: DEBUG timing setup
    cyhal_gpio_init(P12_6, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // Create FreeRTOS task
    xTaskCreate(
        [](void *params) -> void { static_cast<IMU *>(params)->imuTask(); },
        "imuTask",
        4 * configMINIMAL_STACK_SIZE,
        this,
        IMU_TASK_PRIORITY,
        &imuTaskHandle);
}

auto IMU::imuTask() -> void {
    std::array<uint8_t, 4> rawTimestamp{}; // Raw timestamp bytes
    // std::array<uint8_t, 15> rawSensorData{}; // All raw sensor data
    std::array<uint8_t, 12> rawSensorData{}; // All raw sensor data

    std::array<int16_t, 3> xlData{};
    std::array<int16_t, 3> gData{};

    // Initialize last wake up time
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Get data when woken up
    while (true) {
        // Trigger an IMU read every READ_INTERVAL_TICKS ticks
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(READ_INTERVAL_MS));
        // DEBUG: Timing start
        cyhal_gpio_toggle(P12_6);

        // Read all sensor data
        i2cBus->i2cRead1ByteReg(IMU_ADDR, SENSOR_DATA_BEGIN, rawSensorData);

        // Reconstruct signed gyro data
        gData[0] = (rawSensorData[1] << 8) | rawSensorData[0];
        gData[1] = (rawSensorData[3] << 8) | rawSensorData[2];
        gData[2] = (rawSensorData[5] << 8) | rawSensorData[4];

        // Create gyroscope data struct and send
        GyroscopeData gd = {
            .Gx = static_cast<float>(gData[0]) * GYRO_SCALE,
            .Gy = static_cast<float>(gData[1]) * GYRO_SCALE,
            .Gz = static_cast<float>(gData[2]) * GYRO_SCALE,
            .Gts = READ_INTERVAL_MS};

        sendGyroData(gd);

        // Reconstruct signed accel data
        xlData[0] = (rawSensorData[7] << 8) | rawSensorData[6];
        xlData[1] = (rawSensorData[9] << 8) | rawSensorData[8];
        xlData[2] = (rawSensorData[11] << 8) | rawSensorData[10];

        // Create accelerometer data struct and send
        AccelerometerData ad = {
            .Ax = static_cast<float>(xlData[0]) * ACCEL_SCALE,
            .Ay = static_cast<float>(xlData[1]) * ACCEL_SCALE,
            .Az = static_cast<float>(xlData[2]) * ACCEL_SCALE,
            .Ats = READ_INTERVAL_MS};

        sendAccelData(ad);

        // DEBUG: Timing end
        cyhal_gpio_toggle(P12_6);
    }
}
} // namespace IMU
} // namespace Hardware
