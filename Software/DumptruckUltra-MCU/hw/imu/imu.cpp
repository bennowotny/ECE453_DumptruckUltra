#include "hw/imu/imu.hpp"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cy_result.h"
#include "cy_utils.h"
#include "cyhal_gpio.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "cyhal_system.h"
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
         std::function<void(const GyroscopeData &)> sendGyroData,
         std::function<void(const char *)> printStr)
    : i2cBus{std::move(i2cBus)},
      sendAccelData{std::move(sendAccelData)},
      sendGyroData{std::move(sendGyroData)},
      printStr{std::move(printStr)},
      avg_xl_x_off{0},
      avg_xl_y_off{0},
      avg_xl_z_off{0},
      avg_g_x_off{0},
      avg_g_y_off{0},
      avg_g_z_off{0} {
    // TODO: take address as parameter

    // Wait for IMU to boot up
    cyhal_system_delay_ms(15);

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

    // Set low pass filter amount
    dataToSend[0] = XL_SET_LPF2_ODR_OVER_45;
    this->i2cBus->i2cWrite1ByteReg(IMU_ADDR, CTRL8_XL, dataToSend);

    // Configure BYPASS FIFO (clear data)
    this->i2cBus->i2cWrite1ByteReg<1>(IMU_ADDR, FIFO_CTRL4, {0x00});

    // FIXME: DEBUG timing setup
    cyhal_gpio_init(P12_6, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // Create FreeRTOS task
    xTaskCreate(
        [](void *params) -> void { static_cast<IMU *>(params)->imuTask(); },
        "imuTask",
        6 * configMINIMAL_STACK_SIZE,
        this,
        IMU_TASK_PRIORITY,
        &imuTaskHandle);
}

auto IMU::imuTask() -> void {
    // std::array<uint8_t, 15> rawSensorData{}; // All raw sensor data
    std::array<uint8_t, 12> rawSensorData{}; // All raw sensor data

    std::array<int16_t, 3> xlData{};
    std::array<int16_t, 3> gData{};

    // Calibrate
    calibrate();

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
            .Gx = static_cast<float>(gData[0]) * GYRO_SCALE - avg_g_x_off,
            .Gy = static_cast<float>(gData[1]) * GYRO_SCALE - avg_g_y_off,
            .Gz = static_cast<float>(gData[2]) * GYRO_SCALE - avg_g_z_off,
            .Gts = static_cast<float>(READ_INTERVAL_MS) / 1000.0F};

        sendGyroData(gd);

        // Reconstruct signed accel data
        xlData[0] = (rawSensorData[7] << 8) | rawSensorData[6];
        xlData[1] = (rawSensorData[9] << 8) | rawSensorData[8];
        xlData[2] = (rawSensorData[11] << 8) | rawSensorData[10];

        // Create accelerometer data struct and send
        AccelerometerData ad = {
            .Ax = (static_cast<float>(xlData[0]) * ACCEL_SCALE) - avg_xl_x_off /*- 0.075F*/,
            .Ay = (static_cast<float>(xlData[1]) * ACCEL_SCALE) - avg_xl_y_off /*- 0.138F*/,
            .Az = static_cast<float>(xlData[2]) * ACCEL_SCALE - avg_xl_z_off,
            .Ats = static_cast<float>(READ_INTERVAL_MS) / 1000.0F};

        sendAccelData(ad);

        // DEBUG: Timing end
        cyhal_gpio_toggle(P12_6);
    }
}

auto IMU::calibrate() -> void {
    std::array<uint8_t, 12> rawSensorData{}; // All raw sensor data
    std::array<int16_t, 3> xlData{};
    std::array<int16_t, 3> gData{};

    float tot_xl_x = 0.0F;
    float tot_xl_y = 0.0F;
    float tot_xl_z = 0.0F;
    float tot_g_x = 0.0F;
    float tot_g_y = 0.0F;
    float tot_g_z = 0.0F;

    // Wait for readings to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    // Calculate average offset pass 1
    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read all sensor data
        i2cBus->i2cRead1ByteReg(IMU_ADDR, SENSOR_DATA_BEGIN, rawSensorData);

        // Reconstruct signed gyro data
        gData[0] = (rawSensorData[1] << 8) | rawSensorData[0];
        gData[1] = (rawSensorData[3] << 8) | rawSensorData[2];
        gData[2] = (rawSensorData[5] << 8) | rawSensorData[4];

        // Create gyroscope data struct and send
        tot_g_x += static_cast<float>(gData[0]) * GYRO_SCALE;
        tot_g_y += static_cast<float>(gData[1]) * GYRO_SCALE;
        tot_g_z += static_cast<float>(gData[2]) * GYRO_SCALE;

        // Reconstruct signed accel data
        xlData[0] = (rawSensorData[7] << 8) | rawSensorData[6];
        xlData[1] = (rawSensorData[9] << 8) | rawSensorData[8];
        xlData[2] = (rawSensorData[11] << 8) | rawSensorData[10];

        // Create accelerometer data struct and send
        tot_xl_x += (static_cast<float>(xlData[0]) * ACCEL_SCALE);
        tot_xl_y += (static_cast<float>(xlData[1]) * ACCEL_SCALE);
        tot_xl_z += static_cast<float>(xlData[2]) * ACCEL_SCALE;
    }

    // Calculate average samples
    avg_xl_x_off = tot_xl_x / NUM_CALIBRATION_SAMPLES;
    avg_xl_y_off = tot_xl_y / NUM_CALIBRATION_SAMPLES;
    avg_xl_z_off = tot_xl_z / NUM_CALIBRATION_SAMPLES;
    avg_g_x_off = tot_g_x / NUM_CALIBRATION_SAMPLES;
    avg_g_y_off = tot_g_y / NUM_CALIBRATION_SAMPLES;
    avg_g_z_off = tot_g_z / NUM_CALIBRATION_SAMPLES;

    tot_g_x = 0.0F;
    tot_g_y = 0.0F;
    tot_g_z = 0.0F;
    tot_xl_x = 0.0F;
    tot_xl_y = 0.0F;
    tot_xl_z = 0.0F;

    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read all sensor data
        i2cBus->i2cRead1ByteReg(IMU_ADDR, SENSOR_DATA_BEGIN, rawSensorData);

        // Reconstruct signed gyro data
        gData[0] = (rawSensorData[1] << 8) | rawSensorData[0];
        gData[1] = (rawSensorData[3] << 8) | rawSensorData[2];
        gData[2] = (rawSensorData[5] << 8) | rawSensorData[4];

        // Create gyroscope data struct and send
        tot_g_x += static_cast<float>(gData[0]) * GYRO_SCALE - avg_g_x_off;
        tot_g_y += static_cast<float>(gData[1]) * GYRO_SCALE - avg_g_y_off;
        tot_g_z += static_cast<float>(gData[2]) * GYRO_SCALE - avg_g_z_off;

        // Reconstruct signed accel data
        xlData[0] = (rawSensorData[7] << 8) | rawSensorData[6];
        xlData[1] = (rawSensorData[9] << 8) | rawSensorData[8];
        xlData[2] = (rawSensorData[11] << 8) | rawSensorData[10];

        // Create accelerometer data struct and send
        tot_xl_x += (static_cast<float>(xlData[0]) * ACCEL_SCALE) - avg_xl_x_off;
        tot_xl_y += (static_cast<float>(xlData[1]) * ACCEL_SCALE) - avg_xl_y_off;
        tot_xl_z += (static_cast<float>(xlData[2]) * ACCEL_SCALE) - avg_xl_z_off;
    }

    avg_xl_x_off += tot_xl_x / NUM_CALIBRATION_SAMPLES;
    avg_xl_y_off += tot_xl_y / NUM_CALIBRATION_SAMPLES;
    avg_xl_z_off += tot_xl_z / NUM_CALIBRATION_SAMPLES;
    avg_g_x_off += tot_g_x / NUM_CALIBRATION_SAMPLES;
    avg_g_y_off += tot_g_y / NUM_CALIBRATION_SAMPLES;
    avg_g_z_off += tot_g_z / NUM_CALIBRATION_SAMPLES;

    printStr("Calibration complete\r\n");
}
} // namespace IMU
} // namespace Hardware
