#ifndef LOGIC_VISION_OBJECTDETECTOR_HPP
#define LOGIC_VISION_OBJECTDETECTOR_HPP

#include "DeadReckoning.hpp"
#include "proc/proc_setup.hpp"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_uart.h"

namespace Logic {
namespace Vision {

class ObjectDetector {
public:
    [[nodiscard]] auto detectedObject() const -> bool;
    [[nodiscard]] auto currentObjectLocation() const -> DeadReckoning::Pose2D; // WORLD FRAME


private:
    auto uart_init_event_irq() -> void;
    auto uart_pi_handler(cyhal_uart_event_t event) -> void;

    // private config variables
    static constexpr uint32_t BAUD_RATE{115200};
    static constexpr uint32_t UART_DELAY{10u};
    static constexpr uint32_t INT_PRIORITY{3};
    static constexpr uint32_t DATA_BITS{8};
    static constexpr uint32_t STOP_BITS{1};
    static constexpr size_t RX_BUF_SIZE{8};  // 2 floats * 4 bytes/float
    static constexpr uint32_t PACKET_SIZE{2};   // 2 floats (x distance & y distance)
    static constexpr cyhal_gpio_t UART_TX_PIN{Hardware::Processor::MCU_TX_PI_RX};
    static constexpr cyhal_gpio_t UART_RX_PIN{Hardware::Processor::MCU_RX_PI_TX};

    // private variables
    bool DETECT_OBJECT{false};
    float pi_packet[PACKET_SIZE];           // stores floats from converted rx_buf
    uint8_t rx_buf[RX_BUF_SIZE];            // stores bytes from uart
    cyhal_uart_t uart_obj;                  // uart object
};

} // namespace Vision
} // namespace Logic

#endif