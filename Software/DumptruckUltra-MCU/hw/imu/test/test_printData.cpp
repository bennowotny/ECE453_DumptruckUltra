#include "cy_retarget_io.h"
#include "cy_utils.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"
#include <memory>

void printAccelData(const Hardware::IMU::AccelerometerData &aData) {
    static float totalAccel = 0.0F;
    static float numPoints = 0.0F;
    totalAccel += aData.Ax;
    numPoints += 1.0F;
    float average = totalAccel / numPoints;
    (void)average;
    printf("AccelData:\r\n\tx %f\r\n\ty %f\r\n\tz %f\r\n\ttimestamp %f\r\n", aData.Ax, aData.Ay, aData.Az, aData.Ats);
}

void printGyroData(const Hardware::IMU::GyroscopeData &gData) {
    // printf("GyroData:\r\n\tx %f\r\n\ty %f\r\n\tz %f\r\n\ttimestamp %f\r\n", gData.Gx, gData.Gy, gData.Gz, gData.Gts);
}

void printRawData(int16_t data) {
    // printf("%d\r\n", data);
}

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;

    Hardware::Processor::setupProcessor();

    cy_retarget_io_init(P5_1, P5_0, 115200);

    printf("Starting IMU Data\r\n");

    Hardware::I2C::i2cPin_t i2cPins = {.sda = P10_1, .scl = P10_0};
    auto i2cBus{std::make_shared<I2CBusManager>(i2cPins)};

    auto imu(std::make_unique<IMU>(i2cBus, printAccelData, printGyroData, printRawData));

    xTaskCreate(
        [](void *params) -> void {
            while (true) {
            }
        },
        "spin",
        configMINIMAL_STACK_SIZE,
        nullptr,
        tskIDLE_PRIORITY + 1,
        nullptr);

    vTaskStartScheduler();

    CY_ASSERT(0); // Should never reach here
}