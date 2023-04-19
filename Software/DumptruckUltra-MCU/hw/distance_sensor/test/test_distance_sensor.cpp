#include "DistanceSensor.hpp"
#include "cy_retarget_io.h"
#include "i2cBusManager.hpp"
#include "proc_setup.hpp"
#include <cstdio>
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    using Hardware::DistanceSensor::DistanceSensor;
    using Hardware::I2C::I2CBusManager;

    const Hardware::I2C::I2CBusManager::i2cPin_t pins{.sda = P9_1, .scl = P9_0};
    auto busManager{std::make_shared<I2CBusManager>(pins)};
    DistanceSensor uut{busManager, 0x52 >> 1};

    cy_retarget_io_init(P6_5, P6_4, 9600);

    printf("Starting ranging...\r\n");

    while (true) {
        printf("Current distance: %0.2f\r\n", uut.getDistanceMeters());
        // uut.getDistanceMeters();
        cyhal_system_delay_ms(500);
    }
}