#include "DistanceSensor.hpp"
#include "cyhal_system.h"
#include "driver/platform/vl53l1_platform.h"
#include <array>

extern "C" {
auto VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    for (uint32_t i{0}; i < count; ++i) {
        busManager.i2cWrite2ByteReg<1>(dev, index, {pdata[i]});
    }
    return 0;
}

auto VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    std::array<uint8_t, 1> temporaryStoarge{};
    for (uint32_t i{0}; i < count; ++i) {
        busManager.i2cRead2ByteReg(dev, index, temporaryStoarge);
        pdata[i] = temporaryStoarge.at(0);
    }
    return 0;
}

auto VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    busManager.i2cWrite2ByteReg<1>(dev, index, {data});
    return 0; // to be implemented
}

auto VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    busManager.i2cWrite2ByteReg<2>(dev, index, {static_cast<uint8_t>((data >> 8) & 0xFF), static_cast<uint8_t>(data & 0xFF)});
    return 0; // to be implemented
}

auto VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    busManager.i2cWrite2ByteReg<4>(dev, index, {static_cast<uint8_t>((data >> 24) & 0xFF), static_cast<uint8_t>((data >> 16) & 0xFF), static_cast<uint8_t>((data >> 8) & 0xFF), static_cast<uint8_t>(data & 0xFF)});
    return 0; // to be implemented
}

auto VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    std::array<uint8_t, 1> tempStorage{};
    busManager.i2cRead2ByteReg(dev, index, tempStorage);
    *data = tempStorage.at(0);
    return 0; // to be implemented
}

auto VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    std::array<uint8_t, 2> tempStorage{};
    busManager.i2cRead2ByteReg(dev, index, tempStorage);
    *data = static_cast<uint16_t>(tempStorage.at(0) << 8) | tempStorage.at(1);
    return 0; // to be implemented
}

auto VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) -> int8_t {
    auto busManager{Hardware::DistanceSensor::DistanceSensor::getI2CBusManager()};
    std::array<uint8_t, 4> tempStorage{};
    busManager.i2cRead2ByteReg(dev, index, tempStorage);
    *data = static_cast<uint32_t>(tempStorage.at(0) << 24) | static_cast<uint32_t>(tempStorage.at(1) << 16) | static_cast<uint32_t>(tempStorage.at(2) << 8) | tempStorage.at(3);
    return 0; // to be implemented
}

auto VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) -> int8_t {
    cyhal_system_delay_ms(wait_ms);
    return 0; // to be implemented
}
}
