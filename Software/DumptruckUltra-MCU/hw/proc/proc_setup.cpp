#include "proc_setup.hpp"
#include "cybsp.h"

namespace Hardware {
namespace Processor {
void setupProcessor() {

    /* Initialize the device and board peripherals */
    const auto result{cybsp_init()};
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
}
} // namespace Processor
} // namespace Hardware