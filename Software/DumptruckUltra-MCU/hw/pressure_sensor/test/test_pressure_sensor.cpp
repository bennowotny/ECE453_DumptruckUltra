#include "cyhal_psoc6_01_43_smt.h"
#include "hw/pressure_sensor/pressure_sensor.hpp"
#include "proc_setup.hpp"
#include <iostream>

using namespace std;

int main() {
    Hardware::Processor::setupProcessor();

    using Hardware::Pressure_Sensor::Pressure;

    // Setup ADC
    Pressure adc(P10_0);
    cout << "ADC Reading: " << adc.read();
    adc.free()
}
