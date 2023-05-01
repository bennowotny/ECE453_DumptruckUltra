#include "cybsp.h"
#include "cyhal_gpio.h"
#include "cyhal_psoc6_01_43_smt.h"
#include <cstdlib>
#include <memory>


/* Interrupt handler callback function */
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event) {
    /* Toggle pin P0_1 on every interrupt */
    cyhal_gpio_toggle(P5_5);
}

class CoolObject {
public:
    int y;
};

auto main() -> int {

    /* Initialize the device and board peripherals */
    const auto result{cybsp_init()};
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
    cy_rslt_t rslt;
    /* Initialize pin P0_0 GPIO as an input pin */
    rslt = cyhal_gpio_init(P5_6, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    /* Initialize pin P0_1 GPIO as an output with strong drive mode and initial value = true (high) */
    rslt = cyhal_gpio_init(P5_5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    /* Register callback function - gpio_interrupt_handler and pass the value global_count */
    cyhal_gpio_callback_data_t data{
        .callback = gpio_interrupt_handler,
        .callback_arg = nullptr};
    cyhal_gpio_register_callback(P5_6, &data);
    /* Enable falling edge interrupt event with interrupt priority set to 3 */
    cyhal_gpio_enable_event(P5_6, CYHAL_GPIO_IRQ_FALL, 3, true);

    int *p = (int *)malloc(sizeof(CoolObject));
    CoolObject c = CoolObject();

    // auto p{std::make_unique<CoolObject>()};
    // (void)p;

    *p = 0;
    (void)c;

    while (true)
        ;
}