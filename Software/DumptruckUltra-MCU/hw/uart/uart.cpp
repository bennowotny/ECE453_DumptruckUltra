/*
 * uart.c
 *
 */
 
#include "uart.hpp"

// Global UART Variables
volatile uint8_t buffer[BUFFER_SIZE];
volatile bool ALERT_STR = false;		        
volatile uint16_t Rx_cnt = 0;					
volatile int counter = 0;
size_t tx_length = 80;

// UART Macros
#define  BAUD_RATE 115200
#define  UART_DELAY 10u
#define TX_BUF_SIZE 4
#define INT_PRIORITY 3
#define DATA_BITS 8
#define STOP_BITS 1


////////////////////////////////////////////
// Initialization of UART and Interrupts
// Registering of all interrupt sources
////////////////////////////////////////////
static void uart_init_event_irq(void){

    cyhal_uart_t uart_obj;
    const cyhal_uart_cfg_t uart_config = {
        .data_bits = DATA_BITS,
        .stop_bits = STOP_BITS,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL, 
        .rx_buffer_size = 0,  // TODO: Implement RX Buffer    
    }

    // Initialize UART
    cy_rslt_t rslt;
    rslt = cyhal_uart_init(&uart_obj, CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, NULL, &uart_config);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // UART Callback handler registration
    cyhal_uart_register_callback(&uart_obj, uart_event_handler, NULL);

    // Enable required UART events
    cyhal_uart_enable_event(
        &uart_obj,
        (cyhal_uart_event_t)(CYHAL_UART_IRQ_TX_DONE | CYHAL_UART_IRQ_TX_ERROR | CYHAL_UART_IRQ_RX_DONE),
        INT_PRIORITY,
        true
    );
}


//////////////////////////////////////////////////////////
// Console Event Handler to receive characters from console
//////////////////////////////////////////////////////////
void console_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
	uint8_t c;
	cy_rslt_t rslt = cyhal_uart_getc(&cy_retarget_io_uart_obj, &c, 0); // changed (uint8_t*)&c to &c
	buffer[Rx_cnt] = c;
	Rx_cnt++;
	rslt = cyhal_uart_putc(&cy_retarget_io_uart_obj,(uint32_t)c);
	if(rslt == CY_RSLT_SUCCESS){
		if((c=='\n') || (c=='\r')){
			ALERT_STR = true;

		}
		/*else {
			// Loop back around to start of buffer if full
			buffer[counter] = c;
			counter = (counter+1)%BUFFER_SIZE;
		}
*/
	}
}


////////////////////////////////////////////
// UART Callback Function
////////////////////////////////////////////
void uart_event_handler(void *handler_arg, cyhal_uart_event_t event){
    (void) handler_arg;
    if((event & CYHAL_UART_IRQ_TX_ERROR) == CYHAL_UART_IRQ_TX_ERROR){
        printf("TX Error");
        //TODO: Handle TX ERROR
    }else if((event & CYHAL_UART_IRQ_TX_DONE) == CYHAL_UART_IRQ_TX_DONE){
        // All Tx data hass been transmitted
    }else if((event & CYHAL_UART_RX_DONE) == CYHAL_UART_RX_DONE){
        // All Rx data has been received
    }
}


//////////////////////////////////////////////////////////
// Console Init IRQ enables and reegisters all interrupt sources
//////////////////////////////////////////////////////////
static void console_init_irq(void)
{
    // Enable Console Rx Interrupts
	/* The UART callback handler registration */
	cyhal_uart_register_callback(&cy_retarget_io_uart_obj, console_event_handler, NULL);

	/* Enable required UART events */
	cyhal_uart_enable_event(
		&cy_retarget_io_uart_obj,
		(cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY),
		INT_PRIORITY_CONSOLE,
		true
	);
}


//////////////////////////////////////////////////////////
// Setup for console retarget IO
//////////////////////////////////////////////////////////
static void console_init_retarget(void)
{

    /* Initialize retarget-io to use the debug UART port, 8N1 */
	// Baud rate is initially set to 115200, 8N1
	cy_retarget_io_init(PIN_CONSOLE_TX,
						PIN_CONSOLE_RX,
						CY_RETARGET_IO_BAUDRATE);
}

//////////////////////////////////////////////////////////
// Console Init to enable the SCB used for console interface
//////////////////////////////////////////////////////////
void console_init(void)
{
	console_init_retarget();
	console_init_irq();
	printf("Hello\n");
}

