
#include "DeadReckoning.hpp"
#include "DrivingAlgorithm.hpp"
#include "FreeRTOSConfig.h"
#include "Motor.hpp"
#include "Positioning.hpp"
#include "cy_retarget_io.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
#include "portmacro.h"
#include "proc_setup.hpp"
#include "projdefs.h"
#include <cmath>
#include <memory>

void printStr(const char *str) {
    printf("%s", str);
}

auto main() -> int {
    using Hardware::I2C::I2CBusManager;
    using Hardware::IMU::IMU;
    using Logic::DeadReckoning::DeadReckoning;
    Hardware::Processor::setupProcessor();

    Logic::DrivingAlgorithm::DriveMotorLayout layout{
        .leftMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M1_FORWARD, .backwardPin = Hardware::Processor::M1_BACKWARD}, Hardware::Motors::MotorDirection::FORWARD},
        .rightMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M2_FORWARD, .backwardPin = Hardware::Processor::M2_BACKWARD}, Hardware::Motors::MotorDirection::REVERSE}};

    Hardware::I2C::i2cPin_t i2cPins{.sda = Hardware::Processor::I2C_SDA, .scl = Hardware::Processor::I2C_SCL};
    auto i2cBus{std::make_shared<I2CBusManager>(i2cPins)};

    auto deadReckoning{std::make_shared<DeadReckoning>()};

    auto uut =
        std::make_unique<Logic::DrivingAlgorithm::DrivingAlgorithm>(
            layout,
            // FOR TESTER: Change this line to simulate obstacles
            []() -> float { constexpr float SIMULATED_DISTANCE_METERS{99}; return SIMULATED_DISTANCE_METERS; },
            // FOR TESTER: Change this line to simulate different current positions
            [deadReckoning]() -> auto{ return deadReckoning->getCurrentPose(); });

    deadReckoning->setMotorSpeedsHandle([uu2{uut.get()}]() -> auto{ return uu2->getPower(); });

    auto imu{std::make_unique<IMU>(
        i2cBus,
        [uu2{deadReckoning.get()}](const Hardware::IMU::AccelerometerData &msg) -> void { uu2->sendAccelerometerMessage(msg); },
        [uut2{deadReckoning.get()}](const Hardware::IMU::GyroscopeData &msg) -> void { uut2->sendGyroscopeMessage(msg); },
        printStr)};

    // FOR TESTER: Change this line to set the target of the driving algorithm
    uut->loadNewTarget({.x = 10, .y = 0, .heading = 0});

    uut->start();

    // Also run blinky so we know if the RTOS is having a problem
    const auto blinky{std::make_unique<Hardware::Processor::FreeRTOSBlinky>(Hardware::Processor::USER_LED)};

    cy_retarget_io_init(P6_5, P6_4, 115200);

    xTaskCreate(
        [](void *params) -> void {
            auto da = static_cast<Logic::DrivingAlgorithm::DrivingAlgorithm *>(params);
            while (true) {
                if (da->getStatus() == Logic::DrivingAlgorithm::DrivingAlgorithmStatus::COMPLETE) {
                    printf("Driving algorithm finished!\r\n");
                    vTaskDelay(portMAX_DELAY);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        },
        "Poll Driving Algorithm",
        2 * configMINIMAL_STACK_SIZE,
        uut.get(),
        tskIDLE_PRIORITY + 1,
        nullptr);

    xTaskCreate(
        [](void *params) -> void {
            auto dr = static_cast<Logic::DeadReckoning::DeadReckoning *>(params);
            while (true) {
                Logic::DeadReckoning::Pose2D currPos = dr->getCurrentPose();
                printf("%f %f %f\n", currPos.x, currPos.y, currPos.heading);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        },
        "Poll DeadReckoning",
        1 * configMINIMAL_STACK_SIZE,
        deadReckoning.get(),
        tskIDLE_PRIORITY + 1,
        nullptr);

    vTaskStartScheduler();

    CY_ASSERT(false); // Do not return from the scheduler
}