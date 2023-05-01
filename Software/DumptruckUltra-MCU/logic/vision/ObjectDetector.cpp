#include "ObjectDetector.hpp"
#include "DeadReckoning.hpp"
#include "vision/ObjectDetector.hpp"

namespace Logic {
namespace Vision {

    ObjectDetector::ObjectDetector() {
        // Initialize UART
        uart_init_event_irq();
    }

    ///////////////////////////////////////////
    // Return true if object is detected
    ////////////////////////////////////////////
    auto ObjectDetector::detectedObject() const -> bool {
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
        return {};
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
        cy_rslt_t rslt = cyhal_uart_init(&uart_obj, UART_TX_PIN, UART_RX_PIN, CYHAL_NC_PIN_VALUE, CYHAL_NC_PIN_VALUE, NULL, &uart_config);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);

        // Set Baud Rate    
        uint32_t actual_baud;
        rslt = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actual_baud);
        CY_ASSERT(CY_RSLT_SUCCESS == rslt);

        // UART Callback handler registration
        cyhal_uart_register_callback(&uart_obj, [](void *callback_arg, cyhal_uart_event_t event)->void { ((ObjectDetector*)callback_arg) -> uart_pi_handler(event);}, this);

        // Enable required UART events
        cyhal_uart_enable_event(&uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE), INT_PRIORITY, true);
    }



    //////////////////////////////////////////////////////////
    // UART-PI Callback Handler
    //////////////////////////////////////////////////////////
    auto ObjectDetector::uart_pi_handler(cyhal_uart_event_t event)->void{
        if((event & CYHAL_UART_IRQ_RX_DONE)==CYHAL_UART_IRQ_RX_DONE){
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
        }
    }

} // namespace Vision

} // namespace Logic