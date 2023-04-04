/*
 * i2cBusManager.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: surao
 */
#include "i2cBusManager.hpp"
#include <memory>

I2CBusManager::I2CBusManager(struct i2cPin_t *i2cPins) {
    cy_rslt_t rslt = i2cInit(i2cPins);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    auto tmp1{std::make_unique<I2CBusManager>(i2cPins)};
    auto tmp2{std::move(tmp1)};
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
    cy_rslt_t rslt = cyhal_i2c_init(&i2cMonarchObj, i2cPins->sda, i2cPins->scl, NULL);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Configure the I2C resource to be monarch
    rslt = cyhal_i2c_configure(&i2cMonarchObj, &i2cMonarchConfig);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    return rslt;
}
