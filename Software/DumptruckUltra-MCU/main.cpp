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
#include "RGBLED.hpp"
#include "Servo.hpp"
#include "arm-inverse-kinematics/ArmControl.hpp"
#include "cy_utils.h"
#include "dispenser/Dispenser.hpp"
#include "hw/proc/proc_setup.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "logic/fsm/DumptruckUltra.hpp"
#include "logic/fsm/fsmStates.hpp"
#include "pressure_sensor/pressure_sensor.hpp"
#include "proc_setup.hpp"
#include "vision/ObjectDetector.hpp"
#include <memory>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    // Make all objects

    /*
     * RGB LED
     */
    const auto rgbLed{std::make_unique<Hardware::RGB_LED::RGBLED>(
        Hardware::RGB_LED::RGBLayout{
            .redPin = Hardware::Processor::USER_RGB_RED,
            .greenPin = Hardware::Processor::USER_RGB_GREEN,
            .bluePin = Hardware::Processor::USER_RGB_BLUE})};

    const auto blinkyLED{Hardware::Processor::FreeRTOSBlinky(Hardware::Processor::USER_LED, 0, "Blinky")};
    (void)blinkyLED; // Unused, doing it's own thing

    /*
     * I2C BUS
     */
    Hardware::I2C::i2cPin_t i2cPins{
        .sda = Hardware::Processor::I2C_SDA,
        .scl = Hardware::Processor::I2C_SCL};

    auto i2cBus{std::make_shared<Hardware::I2C::I2CBusManager>(i2cPins)};

    /*
     * DISTANCE SENSOR
     */
    // const auto distSensor{std::make_unique<Hardware::DistanceSensor::DistanceSensor>(
    //     i2cBus,
    //     0x52 >> 1)};

    /*
     * DEAD RECKONING
     */
    const auto deadReckoning{std::make_unique<Logic::DeadReckoning::DeadReckoning>()};

    /*
     * DRIVING ALGORITHM
     */
    Logic::DrivingAlgorithm::DriveMotorLayout driveLayout{
        .leftMotor = Hardware::Motors::Motor{
            {.forwardPin = Hardware::Processor::M1_FORWARD, .backwardPin = Hardware::Processor::M1_BACKWARD},
            Hardware::Motors::MotorDirection::REVERSE}, // TODO: Pick pins
        .rightMotor = Hardware::Motors::Motor{{.forwardPin = Hardware::Processor::M2_FORWARD, .backwardPin = Hardware::Processor::M2_BACKWARD}, Hardware::Motors::MotorDirection::REVERSE}};

    const auto drivingAlg{std::make_unique<Logic::DrivingAlgorithm::DrivingAlgorithm>(
        driveLayout,
        // [distSensor{distSensor.get()}]() -> float { return distSensor->getDistanceMeters(); },
        []() -> float { return 1; },
        [deadReckoning{deadReckoning.get()}]() -> Logic::DeadReckoning::Pose2D { return deadReckoning->getCurrentPose(); })};

    deadReckoning->setMotorSpeedsHandle(
        [drivingAlg{drivingAlg.get()}]() { return drivingAlg->getPower(); });

    /*
     * IMU
     */
    const auto imu{std::make_unique<Hardware::IMU::IMU>(
        i2cBus,
        [deadReckoning{deadReckoning.get()}](const Hardware::IMU::AccelerometerData &accelData) { deadReckoning->sendAccelerometerMessage(accelData); },
        [deadReckoning{deadReckoning.get()}](const Hardware::IMU::GyroscopeData &gyroData) { deadReckoning->sendGyroscopeMessage(gyroData); },
        nullptr)}; // FIXME This parameter should not exist

    /*
     * PRESSURE SENSOR
     */
    const auto pressureSensor{std::make_unique<Hardware::PressureSensor::PressureSensor>(Hardware::Processor::PRESSURE_SENSOR_ADC)};

    /*
     * ARM CONTROL
     */
    Logic::Arm::ArmLayout armLayout{
        .shoulder = Hardware::Servos::Servo{Hardware::Processor::SERVO1_PWM},
        .elbow = Hardware::Servos::Servo{Hardware::Processor::SERVO2_PWM},
        .wrist = Hardware::Servos::Servo{Hardware::Processor::SERVO3_PWM},
        .claw = Hardware::Servos::Servo{Hardware::Processor::SERVO4_PWM}};

    const auto arm{std::make_unique<Logic::Arm::ArmControl>(
        armLayout,
        [pressureSensor{pressureSensor.get()}]() -> bool { return pressureSensor->isPressed(); })};

    /*
     * VISION SYSTEM
     */
    const auto vision{std::make_unique<Logic::Vision::ObjectDetector>()};

    /*
     * DISPENSER
     */
    Hardware::Servos::Servo dispenserServo{Hardware::Processor::SERVO5_PWM};
    const auto dispenser{std::make_unique<Logic::Dispenser::Dispenser>(dispenserServo)};

    // Create FSM and add states
    auto dumptruckFSM = std::make_unique<Logic::FSM::DumptruckUltra>(*rgbLed);
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::INIT,
        // {.stateAction{[distSensor{distSensor.get()}]() -> Logic::FSM::DumptruckUltra::FSMState {
        //      return Logic::FSM::initStateAction([distSensor]() { distSensor->init(); });
        //  }},
        {.stateAction = []() { return Logic::FSM::initStateAction([]() {}); },
         .color{Hardware::RGB_LED::PredefinedColors::BLUE}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DRIVE_TO_SEARCH,
        {.stateAction{[drivingAlg{drivingAlg.get()}]() {
             return Logic::FSM::driveToSearchAction(*drivingAlg);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::CYAN}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::LOCAL_SEARCH,
        {.stateAction{[motorControl{
                           [drivingAlg{drivingAlg.get()}](Logic::DrivingAlgorithm::MotorSpeeds speeds) {
                               drivingAlg->setPower(speeds);
                           }},
                       vision{vision.get()}]() {
             return Logic::FSM::localSearchAction(motorControl, *vision);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::GREEN}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::APPROACH,
        {.stateAction{[drivingAlg{drivingAlg.get()}, vision{vision.get()}]() {
             return Logic::FSM::approachAction(*drivingAlg, *vision);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::MAGENTA}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::PICKUP,
        {.stateAction{[arm{arm.get()}, vision{vision.get()}, deadReckoning{deadReckoning.get()}, dispenser{dispenser.get()}]() {
             return Logic::FSM::pickupAction(*arm, *vision, *deadReckoning, *dispenser);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::RED}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DRIVE_TO_START,
        {.stateAction{[drivingAlg{drivingAlg.get()}]() {
             return Logic::FSM::driveToStartAction(*drivingAlg);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::YELLOW}});
    dumptruckFSM->addToStateTable(
        Logic::FSM::DumptruckUltra::FSMState::DISPENSE,
        {.stateAction{[dispenser{dispenser.get()}]() {
             return Logic::FSM::dispenseAction(*dispenser);
         }},
         .color{Hardware::RGB_LED::PredefinedColors::WHITE}});

    vTaskStartScheduler();

    CY_ASSERT(0); // Should never reach this
}

/* [] END OF FILE */