/*
 * i2cBusManager.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: surao
 */

#ifndef I2CBUSMANAGER_HPP_
#define I2CBUSMANAGER_HPP_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include <array>

namespace Hardware {
namespace I2C {
struct i2cPin_t {
    cyhal_gpio_t sda;
    cyhal_gpio_t scl;
};

class I2CBusManager {
public:
    I2CBusManager(const i2cPin_t &i2cPins);

    /**
     * One Byte Register Index
     */
    template <std::size_t N>
    void i2cWrite1ByteReg(uint16_t devAddr, uint8_t reg, const std::array<uint8_t, N> &data) {
        // Acquire mutex
        // Write register
        std::array<uint8_t, N + 1> buf{};
        buf.at(0) = reg;
        std::copy(data.begin(), data.end(), buf.begin() + 1);

        cy_rslt_t rslt = cyhal_i2c_master_write(
            &i2cMonarchObj,
            devAddr,
            buf.data(),
            buf.size(),
            I2C_TIMEOUT_MS,
            true);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        // Release mutex
    }

    /**
     * Two Byte Register Index
     */
    template <std::size_t N>
    void i2cWrite2ByteReg(uint16_t devAddr, uint16_t reg, const std::array<uint8_t, N> &data) {
        // Acquire mutex
        // Write register
        std::array<uint8_t, N + 2> buf{};
        buf.at(0) = (reg >> 8) & 0xFF;
        buf.at(1) = (reg)&0xFF;
        std::copy(data.begin(), data.end(), buf.begin() + 2);

        cy_rslt_t rslt = cyhal_i2c_master_write(
            &i2cMonarchObj,
            devAddr,
            buf.data(),
            buf.size(),
            I2C_TIMEOUT_MS,
            true);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        // Release mutex
    }

    /**
     * One Byte Register Index
     */
    template <std::size_t N>
    void i2cRead1ByteReg(uint16_t devAddr, uint8_t reg, std::array<uint8_t, N> &data) {
        // Acquire mutex
        // Read register
        cy_rslt_t rslt = cyhal_i2c_master_write(
            &i2cMonarchObj,
            devAddr,
            &reg,
            1,
            100,
            false);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        rslt = cyhal_i2c_master_read(
            &i2cMonarchObj,
            devAddr,
            data.data(),
            data.size(),
            I2C_TIMEOUT_MS,
            true);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        // Release mutex
    }

    /**
     * Two Byte Register Index
     */
    template <std::size_t N>
    void i2cRead2ByteReg(uint16_t devAddr, uint16_t reg, std::array<uint8_t, N> &data) {
        // Acquire mutex
        // Read register
        uint8_t arr[2] = {static_cast<uint8_t>((reg >> 8) & 0xFF), static_cast<uint8_t>(reg & 0xFF)};
        cy_rslt_t rslt = cyhal_i2c_master_write(
            &i2cMonarchObj,
            devAddr,
            arr,
            2,
            100,
            false);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        rslt = cyhal_i2c_master_read(
            &i2cMonarchObj,
            devAddr,
            data.data(),
            data.size(),
            I2C_TIMEOUT_MS,
            true);

        CY_ASSERT(rslt == CY_RSLT_SUCCESS);

        // Release mutex
    }

    // TODO: Add a get task handle function

private:
    static constexpr uint32_t I2C_MASTER_FREQUENCY_HZ{100000};
    static constexpr uint32_t I2C_TIMEOUT_MS{100};
    cyhal_i2c_t i2cMonarchObj;
    cy_rslt_t i2cInit(const i2cPin_t &i2cPins);
};

} // namespace I2C
} // namespace Hardware

#endif /* I2CBUSMANAGER_HPP_ */
