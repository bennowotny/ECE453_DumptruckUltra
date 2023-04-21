#include "cy_retarget_io.h"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"
#include <functional>
#include <memory>

void printAccelData(const Hardware::IMU::AccelerometerData &aData) {
    // printf("AccelData:\r\n\tx %f\r\n\ty %f\r\n\tz %f\r\n\ttimestamp %f\r\n", aData.Ax, aData.Ay, aData.Ay, aData.Ats);
}

void printGyroData(const Hardware::IMU::GyroscopeData &gData) {
    printf("GyroData:\r\n\tx %f\r\n\ty %f\r\n\tz %f\r\n\ttimestamp %f\r\n", gData.Gx, gData.Gy, gData.Gy, gData.Gts);
}
    
void printError(uint8_t tag) {
    printf("Unknown tag found: %x\r\n", tag);
}

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;
    Hardware::Processor::setupProcessor();

    cy_retarget_io_init(P5_1, P5_0, 115200);
    setbuf(stdout, nullptr);
    printf("Starting IMU\n");

    I2CBusManager::i2cPin_t i2cPins = {.sda = P10_1, .scl = P10_0};
    auto i2cBus(std::make_shared<I2CBusManager>(i2cPins));
    auto imu(std::make_unique<IMU>(i2cBus, printAccelData, printGyroData, printError));

    vTaskStartScheduler();

    CY_ASSERT(0); // Should never reach here
}
