#include "hw/i2c/i2cBusManager.hpp"
#include "hw/proc/proc_setup.hpp"

int main() {
    Hardware::Processor::setupProcessor();

    // Setup I2C
    I2CBusManager::i2cPin_t i2cPins = {.sda = P5_1, .scl = P5_0};
    I2CBusManager i2cBus(&i2cPins);

    uint8_t value = 0x00;
    i2cBus.i2cWriteReg(0x20, 0x03, &value, 1);

    i2cBus.i2cWriteReg(0x20, 0x02, &value, 1);

    value = 0x01;
    i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
    cyhal_system_delay_ms(500);

    uint8_t read_val;

    // Blink lights on GPIO expander
    while (1) {
        i2cBus.i2cReadReg(0x20, 0x01, &read_val, 1);

        if (read_val == 0x01)
            value = 0x40;
        else
            value = read_val >> 1;

        i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
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