#include "hw/imu/imu.hpp"
#include "FreeRTOS.h"
#include "cy_result.h"
#include "cy_utils.h"
#include "cyhal_gpio.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
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

    // Set up interrupt pin
    cy_rslt_t res;
    res = cyhal_gpio_init(IMU_INT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Set up interrupt callback
    cyhal_gpio_callback_data_t imuGetDataCallback =
        {
            .callback = [](void *param, cyhal_gpio_event_t event) -> void { static_cast<IMU *>(param)->getDataCallback(); },
            .callback_arg = this};
    cyhal_gpio_register_callback(IMU_INT_PIN, &imuGetDataCallback);

    // Disable I3C
    std::array<uint8_t, 1> dataToSend{XL_DISABLE_I3C};
    i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL9_XL, dataToSend);

    // Register address auto-incremented on multi-byte read by default

    // Enable block data update
    dataToSend[0] = SET_BLK_DATA_UPDATE;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, CTRL3_C, dataToSend);

    // Select FIFO Mode
    dataToSend[0] = CONT_MODE;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, FIFO_CTRL4, dataToSend);

    // Set BDR counter threshold. Two register control this threshold. First reg
    // should always be 0 unless we need a threshold over 255.
    dataToSend[0] = CNT_BDR_TH;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, COUNTER_BDR_REG1, {0x00});
    i2cBus->i2cWriteReg<1>(IMU_ADDR, COUNTER_BDR_REG2, dataToSend);

    // Configure interrupt generation on IMU
    dataToSend[0] = EN_BDR_INT;
    i2cBus->i2cWriteReg<1>(IMU_ADDR, INT1_CTRL, dataToSend);

    // Enable both the accelerometer and gyroscope
    dataToSend[0] = XL_CTRL;
    i2cBus->i2cWriteReg(IMU_ADDR, CTRL1_XL, dataToSend);
    dataToSend[0] = G_CTRL;
    i2cBus->i2cWriteReg(IMU_ADDR, CTRL2_G, dataToSend);

    // Check device ID

    // Create FreeRTOS task
}

auto IMU::getDataCallback() -> void {
}
} // namespace IMU
} // namespace Hardware