#include "BareDriver/platform/inc/vl53l1_platform.h"
#include "i2cBusManager.hpp"
#include "vl53l1_error_codes.h"

Hardware::I2C::I2CBusManager busManager{{.sda = P0_0, .scl = P0_0}};

extern "C" {
extern auto VL53L1_CommsInitialise(
    VL53L1_Dev_t *pdev,
    uint8_t comms_type,
    uint16_t comms_speed_khz) -> VL53L1_Error {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
    busManager.i2cReadReg(uint16_t devAddr, uint8_t reg, std::array<uint8_t, N> & data);
}

extern VL53L1_Error VL53L1_CommsClose(
    VL53L1_Dev_t *pdev) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WriteMulti(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_ReadMulti(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WrByte(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint8_t data) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WrWord(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint16_t data) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WrDWord(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint32_t data) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_RdByte(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_RdWord(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint16_t *pdata) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_RdDWord(
    VL53L1_Dev_t *pdev,
    uint16_t index,
    uint32_t *pdata) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WaitUs(
    VL53L1_Dev_t *pdev,
    int32_t wait_us) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WaitMs(
    VL53L1_Dev_t *pdev,
    int32_t wait_ms) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

extern VL53L1_Error VL53L1_WaitValueMaskEx(
    VL53L1_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms) {
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}
}
