/*
* uart.h
*
*/

#ifndef UART_H_
#define UART_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
// #include "cy_retarget_io.h"

// UART Pin Configurations
#define UART_TX P5_1
#define UART_RX P5_0

// Global Variables
#define BUFFER_SIZE 80
extern volatile uint8_t buffer[];		// storage for input characters
extern volatile int counter;
extern volatile uint16_t Rx_cnt;
extern volatile bool ALERT_STR;// keeps track of next buffer (re)placement slot

#define DEBUG_MESSAGE_MAX_LEN   (100u)
#define INT_PRIORITY_CONSOLE	3


/* Public Function API */
void console_init(void);


#endif /* CONSOLE_H_ */
