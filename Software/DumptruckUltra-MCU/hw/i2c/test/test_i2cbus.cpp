#include "hw/i2c/i2cBusManager.hpp"
#include "hw/proc/proc_setup.hpp"

int main() {
    Hardware::Processor::setupProcessor();

    // Setup I2C
    I2CBusManager i2cBus(PIN_MCU_SDA, PIN_MCU_SCL);

    uint8_t value = 0x00;
    i2cBus.i2cWriteReg(0x20, 0x03, &value, 1);

    // Blink lights on GPIO expander
    while(1) {
        value = 0x00;
        i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
        cyhal_system_delay_ms(500);

        value = 0xFF;
        i2cBus.i2cWriteReg(0x20, 0x01, &value, 1);
        cyhal_system_delay_ms(500);
    }
}