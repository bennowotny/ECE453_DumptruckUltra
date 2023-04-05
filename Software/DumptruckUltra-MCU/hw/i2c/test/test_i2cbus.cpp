#include "hw/i2c/i2cBusManager.hpp"
#include "hw/proc/proc_setup.hpp"

int main() {
    Hardware::Processor::setupProcessor();

    // Setup I2C
    I2CBusManager::i2cPin_t i2cPins{.sda = P5_1, .scl = P5_0};
    I2CBusManager i2cBus{i2cPins};

    // uint8_t value = 0x00;
    i2cBus.i2cWriteReg<1>(0x20, 0x03, {0x00});

    i2cBus.i2cWriteReg<1>(0x20, 0x02, {0x00});

    // value = 0x01;
    std::array<uint8_t, 1> readBack;
    i2cBus.i2cWriteReg(0x20, 0x01, readBack);
    cyhal_system_delay_ms(500);

    // uint8_t read_val;

    // Blink lights on GPIO expander
    while (1) {
        i2cBus.i2cReadReg(0x20, 0x01, readBack);

        if (readBack.at(0) == 0x01)
            readBack.at(0) = 0x40;
        else
            readBack.at(0) = readBack.at(0) >> 1;

        i2cBus.i2cWriteReg(0x20, 0x01, readBack);
        cyhal_system_delay_ms(500);
    }
    // value = 0x30;
    // i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
    // cyhal_system_delay_ms(500);

    // value = 0x40;
    // i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
    // cyhal_system_delay_ms(500);

    // value = 0x20;
    // i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
    // cyhal_system_delay_ms(500);

    // value = 0x10;
    // i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
    // cyhal_system_delay_ms(500);
}