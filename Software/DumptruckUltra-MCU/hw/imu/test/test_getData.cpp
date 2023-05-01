#include "cy_retarget_io.h"
#include "cyhal_gpio.h"
#include "cyhal_gpio_impl.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "gpio_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"
#include <array>
#include <functional>
#include <memory>

void printAccelData(const Hardware::IMU::AccelerometerData &aData) {
    printf("AccelData:\n\tx %f\n\ty %f\n\tz %f\n\ttimestamp %f\n", aData.Ax, aData.Ay, aData.Ay, aData.Ats);
}

void printGyroData(const Hardware::IMU::GyroscopeData &gData) {
    printf("GyroData:\n\tx %f\n\ty %f\n\tz %f\n\ttimestamp %f\n", gData.Gx, gData.Gy, gData.Gy, gData.Gts);
}

void interrupt(void *arg, cyhal_gpio_event_t /* event */) {
    cyhal_gpio_toggle(P5_5);
    // Hardware::I2C::I2CBusManager *const i2cBus = (Hardware::I2C::I2CBusManager *)arg;
    // std::array<uint8_t, 1> dataToRec{};

    // for (std::size_t i{0}; i < 20; ++i) {
    //     // Clear Interrupt
    //     // i2cBus->i2cWrite1ByteReg<2>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::COUNTER_BDR_REG1, {0x40, 0x0A});
    //     i2cBus->i2cRead1ByteReg(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::FIFO_DATA_OUT_TAG, dataToRec);

    //     std::array<uint8_t, 6> rawData{};
    //     i2cBus->i2cRead1ByteReg(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::FIFO_DATA_OUT_BEGIN, rawData);

    //     printf("Sensor ID: %x\r\n", dataToRec.at(0) >> 3);
    //     switch (dataToRec.at(0) >> 3) {
    //     case 0x01:
    //         printf("XL Data Heard!\r\n");
    //         printf("Raw Data: %0.2f %0.2f %0.2f\r\n", (9.81 * 0.061 / 1000) * static_cast<int16_t>((rawData.at(1) << 8) | rawData.at(0)), (9.81 * 0.061 / 1000) * static_cast<int16_t>((rawData.at(3) << 8) | rawData.at(2)), (9.81 * 0.061 / 1000) * static_cast<int16_t>((rawData.at(5) << 8) | rawData.at(4)));
    //         break;
    //     case 0x02:
    //         printf("Gyroscope Data Heard!\r\n");
    //         printf("Raw Data: %0.2f %0.2f %0.2f\r\n", (70.0 / 1000) * static_cast<int16_t>((rawData.at(1) << 8) | rawData.at(0)), (70.0 / 1000) * static_cast<int16_t>((rawData.at(3) << 8) | rawData.at(2)), (70.0 / 1000) * static_cast<int16_t>((rawData.at(5) << 8) | rawData.at(4)));
    //         break;
    //     default:
    //         printf("Unknown Sensor: %x\r\n", dataToRec.at(0));
    //         break;
    //     }
    // }
}

class CoolObject {
public:
    int y;
};

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;
    Hardware::Processor::setupProcessor();

    // Hardware::I2C::i2cPin_t i2cPins = {.sda = P10_1, .scl = P10_0};
    // auto i2cBus = new I2CBusManager(i2cPins);
    // CoolObject *x = new CoolObject();
    CoolObject *x = (CoolObject *)malloc(sizeof(CoolObject));
    CoolObject c = CoolObject();
    *x = c;

    // (void)c;
    // (void)i2cBus;
    /*
    auto imu(std::make_unique<IMU>(i2cBus, printAccelData, printGyroData));

    vTaskStartScheduler();*/

    // // Query WHO_AM_I
    // std::array<uint8_t, 1> dataToRec{};
    // i2cBus->i2cRead1ByteReg(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::WHO_AM_I, dataToRec);
    // CY_ASSERT(dataToRec[0] == Hardware::IMU::IMU::IMU_DEV_ID);

    // Set up interrupt pin
    cy_rslt_t res;
    res = cyhal_gpio_init(P5_6, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(res == CY_RSLT_SUCCESS);

    // Configure interrupt
    cyhal_gpio_callback_data_t imuGetDataCallback =
        {
            .callback = interrupt,
            .callback_arg = nullptr};
    cyhal_gpio_register_callback(P5_6, &imuGetDataCallback);
    cyhal_gpio_enable_event(P5_6, CYHAL_GPIO_IRQ_FALL, 3, true);

    cyhal_gpio_init(P5_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // // Disable I3C
    // std::array<uint8_t, 1> dataToSend;
    // dataToSend[0] = Hardware::IMU::IMU::XL_DISABLE_I3C;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::CTRL9_XL, dataToSend);

    // // Enable block data update && autoincrement on read/write
    // dataToSend[0] = Hardware::IMU::IMU::SET_BLK_DATA_UPDATE | Hardware::IMU::IMU::SET_AUTO_INCREMENT;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::CTRL3_C, dataToSend);

    // // Set XL and G scale
    // // Enable XL and G w/ ODR
    // dataToSend[0] = Hardware::IMU::IMU::XL_CTRL;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::CTRL1_XL, dataToSend);

    // dataToSend[0] = Hardware::IMU::IMU::G_CTRL;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::CTRL2_G, dataToSend);

    // // Set BDR on XL and G
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::FIFO_CTRL3, {0x44});

    // // Configure BYPASS FIFO (clear data)
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::FIFO_CTRL4, {0x00});

    // // // Set BDR counter threshold. Two register control this threshold. First reg
    // // // should always be 0 unless we need a threshold over 255.
    // // dataToSend[0] = Hardware::IMU::IMU::CNT_BDR_TH;
    // // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::COUNTER_BDR_REG1, {0x20});
    // // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::COUNTER_BDR_REG2, dataToSend);

    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, 0x07, {0x14});

    // // Configure interrupt generation on IMU
    // dataToSend.at(0) = Hardware::IMU::IMU::EN_BDR_INT;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::INT1_CTRL, {0x08});

    // // Configure CONTINUOUS FIFO
    // dataToSend[0] = Hardware::IMU::IMU::CONT_MODE;
    // i2cBus->i2cWrite1ByteReg<1>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::FIFO_CTRL4, dataToSend);

    // cy_retarget_io_init(P5_1, P5_0, 9600);

    // // Clear Interrupt
    // i2cBus->i2cWrite1ByteReg<2>(Hardware::IMU::IMU::IMU_ADDR, Hardware::IMU::IMU::COUNTER_BDR_REG1, {0x40, 0x0A});

    while (true) {
        // cyhal_system_delay_ms(1);
    }
    // CY_ASSERT(0); // Should never reach here
}
