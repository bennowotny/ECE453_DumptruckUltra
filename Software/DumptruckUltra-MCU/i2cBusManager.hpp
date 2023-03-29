/*
 * i2cBusManager.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: surao
 */

#ifndef I2CBUSMANAGER_HPP_
#define I2CBUSMANAGER_HPP_

#ifndef PIN_MCU_SDA
#define PIN_MCU_SCL P5_0
#define PIN_MCU_SDA P5_1
#endif // !PIN_MCU_SDA

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include <stdio.h>

/* Macros */
#define I2C_MASTER_FREQUENCY 100000u

/* Public API */

class I2CBusManager {
private:
    cyhal_i2c_t i2cMonarchObj;
    cy_rslt_t i2cInit(cyhal_gpio_t sda, cyhal_gpio_t scl);

public:
    I2CBusManager(cyhal_gpio_t sda, cyhal_gpio_t scl);
    cy_rslt_t i2cWriteReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size);
    cy_rslt_t i2cReadReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size);
    // TODO: Add a get task handle function
};

#endif /* I2CBUSMANAGER_HPP_ */
