/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the Empty PSoC6 Application
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *******************************************************************************/

#include "DeadReckoning.hpp"
#include "DistanceSensor.hpp"
#include "DrivingAlgorithm.hpp"
#include "DumptruckUltra.hpp"
#include "cy_utils.h"
#include "fsmStates.hpp"
#include "hw/proc/proc_setup.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    // Make all objects
    // const auto blinkyLED{Hardware::Processor::FreeRTOSBlinky(Hardware::Processor::USER_LED, 0, "Blinky")};

    Hardware::I2C::I2CBusManager::i2cPin_t i2cPins{
        .sda = Hardware::Processor::I2C_SDA,
        .scl = Hardware::Processor::I2C_SCL};

    auto i2cBus{std::make_shared<Hardware::I2C::I2CBusManager>(&i2cPins)};

    auto deadReckoning{std::make_shared<Logic::DeadReckoning::DeadReckoning>()};

    auto imu{std::make_unique<Hardware::IMU::IMU>(
        i2cBus,
        [deadReckoning](const Hardware::IMU::AccelerometerData &accelData) { deadReckoning->sendAccelerometerMessage(accelData); },
        [deadReckoning](const Hardware::IMU::GyroscopeData &gyroData) { deadReckoning->sendGyroscopeMessage(gyroData); })};

    auto distSensor{std::make_shared<Hardware::DistanceSensor::DistanceSensor>(
        i2cBus,
        0x52 >> 1)};

    Logic::DrivingAlgorithm::DriveMotorLayout layout{
        .leftMotor = Hardware::Motors::Motor{
            {.forwardPin = Hardware::Processor::M1_Forward, .backwardPin = Hardware::Processor::M1_Backward},
            Hardware::Motors::MotorDirection::FORWARD}, // TODO: Pick pins
        .rightMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M2_Forward, .backwardPin = Hardware::Processor::M2_Backward}, Hardware::Motors::MotorDirection::REVERSE}};

    auto drivingAlg{std::make_unique<Logic::DrivingAlgorithm::DrivingAlgorithm>(
        layout,
        [distSensor]() -> float { return distSensor->getDistanceMeters(); },
        [deadReckoning]() -> Logic::DeadReckoning::Pose2D { return deadReckoning->getCurrentPose(); })};

    // Create FSM and add states
    auto dumptruckFSM = std::make_unique<Logic::FSM::DumptruckUltra>();
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::INIT,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::initStateAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DRIVE_TO_SEARCH,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::driveToSearchAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::LOCAL_SEARCH,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::localSearchAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::APPROACH,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::approachAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::PICKUP,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::pickupAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DRIVE_TO_START,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::driveToStartAction(); });
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DISPENSE,
        []() -> Logic::FSM::DumptruckUltra::FSMState { return Logic::FSM::dispenseAction(); });

    vTaskStartScheduler();

    CY_ASSERT(0); // Should never reach this
}

/* [] END OF FILE */