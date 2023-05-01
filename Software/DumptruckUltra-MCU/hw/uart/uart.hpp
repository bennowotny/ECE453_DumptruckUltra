/*
* uart.h
*
*/

#ifndef UART_H_
#define UART_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_uart.h"
#include "proc/proc_setup.hpp"

// UART Configs
#define BAUD_RATE 115200
#define UART_DELAY 10u
#define INT_PRIORITY 3
#define DATA_BITS 8
#define STOP_BITS 1

// DEFINE: UART PINS
#define UART_TX_PIN Hardware::Processor::MCU_TX_PI_RX
#define UART_RX_PIN Hardware::Processor::MCU_RX_PI_TX

// Buffer Configs
const size_t RX_BUF_SIZE = 28;              // 7 floats * 4 bytes/float
const size_t PACKET_SIZE = 7;               // 7 floats

// Global UART Variables
extern volatile bool DETECT_OBJECT;
extern volatile float pi_packet[PACKET_SIZE];
//volatile uint16_t rx_count = 0;	        // use if we want to read in multiple packets

// Public functions
void uart_init_event_irq(void);


#endif /* CONSOLE_H_ */
