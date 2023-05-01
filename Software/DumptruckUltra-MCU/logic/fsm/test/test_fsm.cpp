#include "DumptruckUltra.hpp"
#include "RGBLED.hpp"
#include "cy_retarget_io.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "fsmStates.hpp"
#include "hw/pressure_sensor/pressure_sensor.hpp"
#include "proc_setup.hpp"
#include "rgb_led/RGBLED.hpp"
#include <memory>

auto main() -> int {

    Hardware::Processor::setupProcessor();

    using Hardware::PressureSensor::PressureSensor;
    using Hardware::RGB_LED::Color;
    using Hardware::RGB_LED::RGBLED;
    using Logic::FSM::DumptruckUltra;

    RGBLED uut{
        {.redPin = Hardware::Processor::USER_RGB_RED,
         .greenPin = Hardware::Processor::USER_RGB_GREEN,
         .bluePin = Hardware::Processor::USER_RGB_BLUE}};

    cy_retarget_io_init(P6_5, P6_4, 9600);
    // PressureSensor adc(P10_0);
    auto adc = std::make_shared<PressureSensor>(P10_0);

    printf("State Machine Start: ");
    // Make all states and make sure transitions between them work
    auto dumptruckFSM = std::make_unique<DumptruckUltra>(uut);
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::INIT,
        {.stateAction = [adc]() -> DumptruckUltra::FSMState {
             vTaskDelay(pdMS_TO_TICKS(500));
             if (adc->isPressed()) {
                 printf("State Transisiton \r\n");
                 return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
             }
             return DumptruckUltra::FSMState::INIT;
         },
         .color = Hardware::RGB_LED::PredefinedColors::BLUE});
    dumptruckFSM->addToStateTable(
        DumptruckUltra::FSMState::DRIVE_TO_SEARCH,
        {.stateAction = [adc]() -> DumptruckUltra::FSMState { 
            vTaskDelay(pdMS_TO_TICKS(500));
            printf("Second State Transisiton \r\n");
            printf("ADC Reading: %ld\r\n", adc->read());
            return DumptruckUltra::FSMState::INIT; },
         .color = Hardware::RGB_LED::PredefinedColors::CYAN});
    vTaskStartScheduler();
}
