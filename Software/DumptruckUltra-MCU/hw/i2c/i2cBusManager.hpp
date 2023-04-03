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
#include <stdio.h>

/* Macros */
#define I2C_MASTER_FREQUENCY 100000u

/* Public API */

class I2CBusManager {
private:
    static constexpr uint32_t I2C_MASTER_FREQUENCY_HZ{100000};
    cyhal_i2c_t i2cMonarchObj;
    cy_rslt_t i2cInit(cyhal_gpio_t sda, cyhal_gpio_t scl);

public:
    I2CBusManager(cyhal_gpio_t sda, cyhal_gpio_t scl);
    void i2cWriteReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size);
    void i2cReadReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size);
    // TODO: Add a get task handle function
};

#endif /* I2CBUSMANAGER_HPP_ */
