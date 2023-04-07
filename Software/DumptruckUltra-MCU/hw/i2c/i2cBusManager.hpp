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
#include <cstddef>
#include <cstdio>

namespace Hardware {
namespace I2C {

class I2CBusManager {
public:
    struct i2cPin_t {
        cyhal_gpio_t sda;
        cyhal_gpio_t scl;
    };

    I2CBusManager(const i2cPin_t &i2cPins);

    template <std::size_t N>
    void i2cWriteReg(uint16_t devAddr, uint8_t reg, const std::array<uint8_t, N> &data) {
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

    template <std::size_t N>
    void i2cReadReg(uint16_t devAddr, uint8_t reg, std::array<uint8_t, N> &data) {
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
