#include "DistanceSensor.hpp"
#include "FreeRTOS.h"
#include "cy_retarget_io.h"
#include "hw/i2c/i2cBusManager.hpp"
#include "hw/proc/proc_setup.hpp"
#include "projdefs.h"
#include "task.h"
#include <memory>

// IO expander address and registers
static constexpr uint8_t IO_EXP_ADDR{0X20};
// static constexpr uint8_t IO_EXP_INREG{0X00};
static constexpr uint8_t IO_EXP_OUTREG{0X01};
static constexpr uint8_t IO_EXP_POLREG{0X02};
static constexpr uint8_t IO_EXP_CONFREG{0X03};

#define LM75_SUBORDINATE_ADDR 0x4F
#define LM75_TEMP_REG 0x00

int main() {
    Hardware::Processor::setupProcessor();

    using Hardware::I2C::I2CBusManager;

    cy_retarget_io_init(P5_1, P5_0, 115200);

    // Setup I2C
    Hardware::I2C::i2cPin_t i2cPins = {.sda = P10_1, .scl = P10_0};
    auto i2cBus{std::make_unique<I2CBusManager>(i2cPins)};

    xTaskCreate(
        [](void *params) {
            auto *i2cBus{static_cast<I2CBusManager *>(params)};

            i2cBus->i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_CONFREG, {0x00});

            i2cBus->i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_POLREG, {0x00});

            i2cBus->i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, {0x01});

            uint8_t value = 0x00;
            std::array<uint8_t, 1> read_val{};

            // Blink lights on GPIO expander
            while (true) {
                i2cBus->i2cRead1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, read_val);

                if (read_val.at(0) == 0x01)
                    value = 0x40;
                else
                    value = read_val.at(0) >> 1;

                i2cBus->i2cWrite1ByteReg<1>(IO_EXP_ADDR, IO_EXP_OUTREG, {value});
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        },
        "GPIO Expander",
        2 * configMINIMAL_STACK_SIZE,
        i2cBus.get(),
        tskIDLE_PRIORITY + 1,
        nullptr);

    xTaskCreate(
        [](void *params) {
            auto *i2cBus{static_cast<I2CBusManager *>(params)};

            vTaskDelay(pdMS_TO_TICKS(500));
            std::array<uint8_t, 2> read_val{};
            int16_t rawTemp = 0;
            float temperature = 0.0F;

            // Blink lights on GPIO expander
            while (true) {
                i2cBus->i2cRead1ByteReg<2>(LM75_SUBORDINATE_ADDR, LM75_TEMP_REG, read_val);

                rawTemp = (read_val[1] << 8) | read_val[0];
                temperature = static_cast<float>(rawTemp) / 2;

                printf("Temp: %f\r\n", temperature);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        },
        "Temp Sensor",
        2 * configMINIMAL_STACK_SIZE,
        i2cBus.get(),
        tskIDLE_PRIORITY + 1,
        nullptr);

    vTaskStartScheduler();
}