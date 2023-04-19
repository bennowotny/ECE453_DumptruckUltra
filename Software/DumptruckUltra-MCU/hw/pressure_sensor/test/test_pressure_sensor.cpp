#include "cy_retarget_io.h"
#include "cyhal_psoc6_01_43_smt.h"
#include "hw/pressure_sensor/pressure_sensor.hpp"
#include "proc_setup.hpp"
#include <iostream>

using namespace std;

int main() {
    Hardware::Processor::setupProcessor();

    using Hardware::PressureSensor::PressureSensor;

    cy_retarget_io_init(P6_5, P6_4, 9600);

    // Setup ADC
    PressureSensor adc(P10_0);
    cout << "ADC Reading: " << adc.read() + "\r\n";
}
