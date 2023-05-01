#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "uart/uart.hpp"
#include "string.h"

int main(void){
    cy_rslt_t rslt = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    __enable_irq();
    uart_init_event_irq();

    for(;;){
        if(DETECT_OBJECT){
            // Do something with pi_packet
            for(unsigned int i=0;i<PACKET_SIZE;++i){
                printf("%f ", pi_packet[i]);
            }
            printf("\n");
            DETECT_OBJECT = false;
        }
    }
    return 0;
}