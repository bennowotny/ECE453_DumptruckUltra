/*
 * uart.c
 *
 */
#include "uart.hpp"

volatile bool DETECT_OBJECT = false;
volatile float pi_packet[PACKET_SIZE];
uint8_t rx_buff[RX_BUF_SIZE];
cyhal_uart_t uart_obj;

//////////////////////////////////////////////////////////
// UART-PI Callback Handler
//////////////////////////////////////////////////////////
void uart_pi_handler(void *callback_arg, cyhal_uart_event_t event) {
    (void)callback_arg;
    if ((event & CYHAL_UART_IRQ_RX_DONE) == CYHAL_UART_IRQ_RX_DONE) {
        // read in all bytes into rx_buf
        cy_rslt_t rslt;
        size_t NUM_BYTES = RX_BUF_SIZE;
        rslt = cyhal_uart_read(&uart_obj, rx_buff, &NUM_BYTES);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);

        // convert from bytes to floats
        for (unsigned int i = 0; i < PACKET_SIZE; ++i) {
            pi_packet[i] = *((float *)(rx_buff + i * sizeof(float)));
        }

        // Alert when packet is received
        DETECT_OBJECT = true;
    }
}

///////////////////////////////////////////
// Initialization of UART and Interrupts
////////////////////////////////////////////
void uart_init_event_irq(void) {
    const cyhal_uart_cfg_t uart_config = {
        .data_bits = DATA_BITS,
        .stop_bits = STOP_BITS,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = rx_buff,
        .rx_buffer_size = RX_BUF_SIZE,
    };
    // Initialize UART
    cy_rslt_t rslt;

    rslt = cyhal_uart_init(&uart_obj, P5_1, P5_0, CYHAL_NC_PIN_VALUE, CYHAL_NC_PIN_VALUE, NULL, &uart_config);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // Set Baud Rate
    uint32_t actual_baud;
    rslt = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actual_baud);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // UART Callback handler registration
    cyhal_uart_register_callback(&uart_obj, uart_pi_handler, NULL);

    // Enable required UART events
    cyhal_uart_enable_event(&uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE), INT_PRIORITY, true);
}
