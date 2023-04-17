#include "hw/i2c/i2cBusManager.hpp"
#include "hw/proc/proc_setup.hpp"

// IO expander address and registers
static constexpr uint8_t IO_EXP_ADDR{0X20};
// static constexpr uint8_t IO_EXP_INREG{0X00};
static constexpr uint8_t IO_EXP_OUTREG{0X01};
static constexpr uint8_t IO_EXP_POLREG{0X02};
static constexpr uint8_t IO_EXP_CONFREG{0X03};

int main() {
    Hardware::Processor::setupProcessor();

    using Hardware::I2C::I2CBusManager;

    // Setup I2C
    I2CBusManager::i2cPin_t i2cPins = {.sda = P5_1, .scl = P5_0};
    I2CBusManager i2cBus(i2cPins);

    i2cBus.i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_CONFREG, {0x00});

    i2cBus.i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_POLREG, {0x00});

    i2cBus.i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, {0x01});
    cyhal_system_delay_ms(500);

    uint8_t value = 0x00;
    std::array<uint8_t, 1> read_val{};

    // Blink lights on GPIO expander
    while (true) {
        i2cBus.i2cRead1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, read_val);

        if (read_val.at(0) == 0x01)
            value = 0x40;
        else
            value = read_val.at(0) >> 1;

        i2cBus.i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, {value});
        cyhal_system_delay_ms(500);
    }
}