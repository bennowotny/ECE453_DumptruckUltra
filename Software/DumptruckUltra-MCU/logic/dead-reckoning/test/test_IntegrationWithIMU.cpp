#include "DeadReckoning.hpp"
#include "FreeRTOSConfig.h"
#include "cy_retarget_io.h"
#include "cy_utils.h"
#include "cyhal_gpio.h"
#include "cyhal_gpio_impl.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "cyhal_system.h"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "projdefs.h"
#include "task.h"
#include <memory>

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;
    using Logic::DeadReckoning::DeadReckoning;

    Hardware::I2C::i2cPin_t i2cPins{.sda = P10_1, .scl = P10_0};
    auto i2cBus{std::make_shared<I2CBusManager>(i2cPins)};

    auto uut{std::make_shared<DeadReckoning>()};

    auto imu{std::make_unique<IMU>(
        i2cBus,
        [uut](const Hardware::IMU::AccelerometerData &msg) -> void { uut->sendAccelerometerMessage(msg); },
        [uut](const Hardware::IMU::GyroscopeData &msg) -> void { uut->sendGyroscopeMessage(msg); })};

    cy_retarget_io_init(P5_1, P5_0, 115200);

    cyhal_gpio_init(P10_3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // Create task which runs DeadReckoning
    xTaskCreate(
        [](void *params) -> void {
            while (true) {
                Logic::DeadReckoning::DeadReckoning *uut{static_cast<DeadReckoning *>(params)};
                vTaskDelay(pdMS_TO_TICKS(500));
                Logic::DeadReckoning::Pose2D currPose = uut->getCurrentPose();
                (void)currPose;
                // cyhal_gpio_write(P10_3, true);
                // printf("%0.2f %0.2f %0.2f\r\n", currPose.x, currPose.y, currPose.heading);
                // cyhal_gpio_write(P10_3, false);
            }
        },
        "dead_reckoning",
        4 * configMINIMAL_STACK_SIZE,
        (void *)uut.get(),
        tskIDLE_PRIORITY + 2,
        nullptr);

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