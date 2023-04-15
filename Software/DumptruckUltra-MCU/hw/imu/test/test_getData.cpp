#include "../imu.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"
#include <functional>
#include <memory>

void printAccelData(const Hardware::IMU::AccelerometerData &aData) {
    printf("AccelData:\n\tx %f\n\ty %f\n\tz %f\n\ttimestamp %f\n", aData.Ax, aData.Ay, aData.Ay, aData.Ats);
}

void printGyroData(const Hardware::IMU::GyroscopeData &gData) {
    printf("GyroData:\n\tx %f\n\ty %f\n\tz %f\n\ttimestamp %f\n", gData.Gx, gData.Gy, gData.Gy, gData.Gts);
}

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;
    Hardware::Processor::setupProcessor();

    I2CBusManager::i2cPin_t i2cPins = {.sda = P9_1, .scl = P9_0};
    I2CBusManager i2cBus(i2cPins);
    IMU imu(std::make_shared<I2CBusManager>(i2cBus), printAccelData, printGyroData);

    vTaskStartScheduler();

    CY_ASSERT(0); // Should never reach here
}
