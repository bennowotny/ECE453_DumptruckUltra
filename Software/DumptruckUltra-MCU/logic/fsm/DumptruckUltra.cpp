#include "DumptruckUltra.hpp"
#include "Motor.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"

namespace FSM {
using namespace Hardware;
using namespace Logic;

DumptruckUltra::DumptruckUltra()
    : i2cBus({.sda = Processor::I2C_SDA, .scl = Processor::I2C_SCL}),
      imu(std::make_shared<I2C::I2CBusManager>(i2cBus), nullptr, nullptr) {}
} // namespace FSM
