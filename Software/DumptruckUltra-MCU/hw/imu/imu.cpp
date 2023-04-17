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

    // Reset sensors
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL1_XL, {0x00});
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL2_G, {0x00});

    // Set up interrupt callback
    cyhal_gpio_callback_data_t imuGetDataCallback =
        {
            .callback = [](void *param, cyhal_gpio_event_t event) -> void { static_cast<IMU *>(param)->dataReadyCallback(); },
            .callback_arg = this};
    cyhal_gpio_register_callback(IMU_INT_PIN, &imuGetDataCallback);

    // Set up interrupt pin
    cy_rslt_t res;
    res = cyhal_gpio_init(IMU_INT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Disable I3C
    std::array<uint8_t, 1> dataToSend{XL_DISABLE_I3C};
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL9_XL, dataToSend);

    // Register address auto-incremented on multi-byte read by default

    // Enable block data update
    dataToSend[0] = SET_BLK_DATA_UPDATE;
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL3_C, dataToSend);

    // Select FIFO Mode
    dataToSend[0] = CONT_MODE;
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, FIFO_CTRL4, dataToSend);

    // Set BDR counter threshold. Two register control this threshold. First reg
    // should always be 0 unless we need a threshold over 255.
    dataToSend[0] = CNT_BDR_TH;
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, COUNTER_BDR_REG1, {0x00});
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, COUNTER_BDR_REG2, dataToSend);

    // Configure interrupt generation on IMU
    dataToSend[0] = EN_BDR_INT;
    this->i2cBus->i2cWriteReg<1>(IMU_ADDR, INT1_CTRL, dataToSend);

    // Check device ID
    std::array<uint8_t, 1> dataToRec{};
    this->i2cBus->i2cReadReg(IMU_ADDR, WHO_AM_I, dataToRec);
    CY_ASSERT(dataToRec[0] == IMU_DEV_ID);

    // Create FreeRTOS task
    xTaskCreate(
        [](void *params) -> void { static_cast<IMU *>(params)->imuTask(); },
        "imuTask",
        configMINIMAL_STACK_SIZE,
        this,
        IMU_TASK_PRIORITY,
        &imuTaskHandle);
}

auto IMU::imuTask() -> void {
    // Enable both the accelerometer and gyroscope
    // Set output data rate (ODR), scaling, and filtering
    std::array<uint8_t, 1> dataToSend{};
    dataToSend[0] = XL_CTRL;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL1_XL, dataToSend);
    dataToSend[0] = G_CTRL;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL2_G, dataToSend);

    std::array<uint8_t, 1> dataToRec{};     // Data read from control registers
    std::array<uint8_t, 4> rawTimestamp{};  // Raw timestamp bytes
    std::array<uint8_t, 6> rawSensorData{}; // All 6 bytes of data from both sensors
    std::array<int16_t, 3> sensorData{};    // Signed data from both IMU sensors

    uint32_t timestamp{0x00000000};
    uint8_t numDataSamples = 0;

    // Get data when woken up
    while (true) {
        // Wait for data to become available. The notification value keeps track
        // of the number of times the interrupt was fired.
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        // Check how much data is in FIFO
        i2cBus->i2cReadReg<1>(IMU_ADDR, FIFO_STATUS1, dataToRec);
        CY_ASSERT(dataToRec[0] >= CNT_BDR_TH);
        numDataSamples = dataToRec[0];
#ifdef DEBUG
        if (dataToRec[0] > 128)
            printf("IMU:: Unread data in IMU FIFO getting large: %d data points", dataToRec[0]);
#endif

        // Read data tag
        i2cBus->i2cReadReg<1>(IMU_ADDR, FIFO_DATA_OUT_TAG, dataToRec);

        // Get timestamp
        i2cBus->i2cReadReg(IMU_ADDR, TIMESTAMP_REGS, rawTimestamp);
        timestamp = (rawTimestamp[0] << 24) | (rawTimestamp[1] << 16) | (rawTimestamp[2] << 8) | rawTimestamp[3];

        // Read data and send it to another task
        for (int i = 0; i < numDataSamples; i++) {
            if (dataToRec[0] == GYRO_DATA_TAG) {
                // Read gyro data
                i2cBus->i2cReadReg<6>(IMU_ADDR, FIFO_DATA_OUT_BEGIN, rawSensorData);

                // Reconstruct signed data
                sensorData[0] = (rawSensorData[1] << 8) | rawSensorData[0];
                sensorData[1] = (rawSensorData[3] << 8) | rawSensorData[2];
                sensorData[2] = (rawSensorData[5] << 8) | rawSensorData[4];

                // Convert data to float
                GyroscopeData gd = {
                    .Gx = static_cast<float>(sensorData[0]),
                    .Gy = static_cast<float>(sensorData[1]),
                    .Gz = static_cast<float>(sensorData[2]),
                    .Gts = static_cast<float>(timestamp / TIMESTAMP_RES)};

                // Send data to other task
                sendGyroData(gd);
            } else if (dataToRec[0] == ACCEL_DATA_TAG) {
                // Read accelerometer data
                i2cBus->i2cReadReg<6>(IMU_ADDR, FIFO_DATA_OUT_BEGIN, rawSensorData);

                // Reconstruct signed data
                sensorData[0] = (rawSensorData[1] << 8) | rawSensorData[0];
                sensorData[1] = (rawSensorData[3] << 8) | rawSensorData[2];
                sensorData[2] = (rawSensorData[5] << 8) | rawSensorData[4];

                // Convert data to float
                AccelerometerData ad = {
                    .Ax = static_cast<float>(sensorData[0]),
                    .Ay = static_cast<float>(sensorData[1]),
                    .Az = static_cast<float>(sensorData[2]),
                    .Ats = static_cast<float>(timestamp / TIMESTAMP_RES)};

                // Send data to other task
                sendAccelData(ad);
            }
#ifdef DEBUG
            else {
                printf("IMU:: Other tag received! %x", dataToRec[0]);
            }
#endif
        }
    }
}

auto IMU::dataReadyCallback() -> void {
    // Wake imuTask when data is ready. This will increment the notification value.
    // When the imuTask will stay awake while the notification value is above zero.
    BaseType_t taskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imuTaskHandle, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
}
} // namespace IMU
} // namespace Hardware
