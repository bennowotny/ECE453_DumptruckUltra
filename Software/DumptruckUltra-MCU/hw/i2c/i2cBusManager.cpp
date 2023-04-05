/*
 * i2cBusManager.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: surao
 */
#include "i2cBusManager.hpp"

I2CBusManager::I2CBusManager(struct i2cPin_t *i2cPins) {
    cy_rslt_t rslt = i2cInit(i2cPins);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
}

// Define the I2C monarch configuration structure

void I2CBusManager::i2cWriteReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size) {
    // Acquire mutex
    // Write register
    uint8_t buf[size + 1];
    buf[0] = reg;
    memcpy(&(buf[1]), data, size);

    cy_rslt_t rslt = cyhal_i2c_master_write(&i2cMonarchObj, devAddr, buf, size + 1, 100, true);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Release mutex
}

void I2CBusManager::i2cReadReg(uint16_t devAddr, uint8_t reg, uint8_t *data, uint8_t size) {
    // Acquire mutex
    // Read register

    cy_rslt_t rslt = cyhal_i2c_master_write(&i2cMonarchObj, devAddr, &reg, 1, 100, false);

    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    rslt = cyhal_i2c_master_read(&i2cMonarchObj, devAddr, data, size, 100, true);

    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Release mutex
}

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t I2CBusManager::i2cInit(i2cPin_t *i2cPins) {
    cyhal_i2c_cfg_t i2cMonarchConfig =
        {
            CYHAL_I2C_MODE_MASTER,
            0, // address is not used for master mode
            I2C_MASTER_FREQUENCY_HZ};

    // Initialize I2C monarch, set the SDA and SCL pins and assign a new clock
    cy_rslt_t rslt = cyhal_i2c_init(&i2cMonarchObj, i2cPins->sda, i2cPins->scl, nullptr);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Configure the I2C resource to be monarch
    rslt = cyhal_i2c_configure(&i2cMonarchObj, &i2cMonarchConfig);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    return rslt;
}
