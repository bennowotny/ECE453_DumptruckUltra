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
#include <stdio.h>

/* Public API */
class I2CBusManager {
public:
    struct i2cPin_t {
        cyhal_gpio_t sda;
        cyhal_gpio_t scl;
    };

    I2CBusManager(i2cPin_t *i2cPins);

    template <std::size_t N>
    void i2cWriteReg(uint16_t devAddr, uint8_t reg, const std::array<uint8_t, N> &data);

    template <std::size_t N>
    void i2cReadReg(uint16_t devAddr, uint8_t reg, std::array<uint8_t, N> &data);
    // TODO: Add a get task handle function

private:
    static constexpr uint32_t I2C_MASTER_FREQUENCY_HZ{100000};
    cyhal_i2c_t i2cMonarchObj;
    cy_rslt_t i2cInit(I2CBusManager::i2cPin_t *i2cPins);
};

#endif /* I2CBUSMANAGER_HPP_ */
