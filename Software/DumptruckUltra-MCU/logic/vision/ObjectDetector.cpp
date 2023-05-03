#include "ObjectDetector.hpp"
#include "DeadReckoning.hpp"
#include "vision/ObjectDetector.hpp"
#include "cyhal_system.h"

namespace Logic {
namespace Vision {

    ObjectDetector::ObjectDetector() {
        // Initialize UART
        uart_init_event_irq();
    }

    ///////////////////////////////////////////
    // Return true if object is detected
    ////////////////////////////////////////////
    auto ObjectDetector::detectedObject() -> bool {
        //cy_rslt_t rslt = cyhal_uart_read_async(&uart_obj, (void*)rx_buf, RX_BUF_SIZE);

        /*
        size_t NUM_BYTES = RX_BUF_SIZE;
        cy_rslt_t rslt = cyhal_uart_read(&uart_obj, rx_buf, &NUM_BYTES);
        cyhal_system_delay_ms(100);

        CY_ASSERT(CY_RSLT_SUCCESS == rslt);
        for(unsigned int i=0;i<PACKET_SIZE;++i)
            pi_packet[i] = *((float*)(rx_buf + i*sizeof(float)));
        float x = pi_packet[0];
        float y = pi_packet[1];
        float distance = sqrt(x*x + y*y);
        if(distance>50 && distance<1000){
            DETECT_OBJECT = true;
            // clear buffer
        }else{
            DETECT_OBJECT = false;
        }
        */
        return DETECT_OBJECT;
    }



    ///////////////////////////////////////////
    // Return estimatedd x,y distance to object
    ////////////////////////////////////////////
    auto ObjectDetector::currentObjectLocation() const -> DeadReckoning::Pose2D {
        DeadReckoning::Pose2D pose = {.x=0, .y=0, .heading=0};
        if (DETECT_OBJECT) {
            pose.x = pi_packet[0];
            pose.y = pi_packet[1];
        }
        return pose;
    }

    ///////////////////////////////////////////
    // Initialization of UART and Interrupts
    ////////////////////////////////////////////
    auto ObjectDetector::uart_init_event_irq() -> void{
        // UART Default configs & Initialize Buffer
        const cyhal_uart_cfg_t uart_config = {
            .data_bits = DATA_BITS,
            .stop_bits = STOP_BITS,
            .parity = CYHAL_UART_PARITY_NONE,
            .rx_buffer = rx_buf,
            .rx_buffer_size = RX_BUF_SIZE,
        };

        // Initialize UART
        cy_rslt_t rslt = cyhal_uart_init(&uart_obj, UART_TX_PIN, UART_RX_PIN, NC, NC, NULL, &uart_config);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);

        // Set Baud Rate    
        uint32_t actual_baud;
        rslt = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actual_baud);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);

        // UART Callback handler registration
        cyhal_uart_register_callback(&uart_obj, [](void *callback_arg, cyhal_uart_event_t event)->void { ((ObjectDetector*)callback_arg) -> uart_pi_handler(); }, this);

        // Enable required UART events
        cyhal_uart_enable_event(&uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE), INT_PRIORITY, true);

        // Begin asynchronous read
        rslt = cyhal_uart_read_async(&uart_obj, (void*)rx_buf, RX_BUF_SIZE);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    }



    //////////////////////////////////////////////////////////
    // UART-PI Callback Handler
    //////////////////////////////////////////////////////////
    auto ObjectDetector::uart_pi_handler()->void{
        DETECT_OBJECT = true;
        // convert from bytes to floats 
        for(unsigned int i=0;i<PACKET_SIZE;++i){
            pi_packet[i] = *((float*)(rx_buf + i*sizeof(float)));
        }

        /*
        if((event & CYHAL_UART_IRQ_RX_FULL)==CYHAL_UART_IRQ_RX_FULL){
            // read in all bytes into rx_buf
            cy_rslt_t rslt;
            size_t NUM_BYTES = RX_BUF_SIZE;
            rslt = cyhal_uart_read(&uart_obj, rx_buf, &NUM_BYTES);
            CY_ASSERT(CY_RSLT_SUCCESS == rslt);

            // convert from bytes to floats 
            for(unsigned int i=0;i<PACKET_SIZE;++i){
                pi_packet[i] = *((float*)(rx_buf + i*sizeof(float)));
            }

            // Alert when packet is received
            DETECT_OBJECT = true;
        }*/
    }

} // namespace Vision
} // namespace Logic
