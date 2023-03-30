/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "i2cBusManager.hpp"

#define TCA9534_SUBORDINATE_ADDR                 0x20

#define TCA9534_INPUT_PORT_ADDR					 0x00
#define TCA9534_OUTPUT_PORT_ADDR				 0x01
#define TCA9534_POLARITY_ADDR					 0x02
#define TCA9534_CONFIG_ADDR						 0x03

int main()
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    I2CBusManager i2cBus(PIN_MCU_SDA, PIN_MCU_SCL);
    uint8_t value;

    while(1) {
        value = 0x00;

        result = i2cBus.i2cWriteReg(TCA9534_SUBORDINATE_ADDR, TCA9534_CONFIG_ADDR, &value, 1);
        if(result != CY_RSLT_SUCCESS)
            CY_ASSERT(0);
        
        cyhal_system_delay_ms(500);
        
        value = 0xFF;

        result = i2cBus.i2cWriteReg(TCA9534_SUBORDINATE_ADDR, TCA9534_OUTPUT_PORT_ADDR, &value, 1);
        if(result != CY_RSLT_SUCCESS)
            CY_ASSERT(0);
        
        cyhal_system_delay_ms(500);
    }

    for (;;)
    {
    }
}

/* [] END OF FILE */