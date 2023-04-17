#pragma once

#include "DeadReckoning.hpp"
#include "DrivingAlgorithm.hpp"
#include "Motor.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"

namespace FSM {
using namespace Hardware;
using namespace Logic;

class DumptruckUltra {
public:
    DumptruckUltra();

private:
    I2C::I2CBusManager i2cBus;
    IMU::IMU imu;

    DrivingAlgorithm::DriveMotorLayout driveBase;
    DeadReckoning::DeadReckoning deadReck;
    DrivingAlgorithm::DrivingAlgorithm driveAlg;
};
} // namespace FSM