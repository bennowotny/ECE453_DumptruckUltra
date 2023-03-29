#include "cyhal.h" // IWYU pragma: keep

namespace Hardware {
namespace Processor {
using cyhal_gpio_pin = decltype(P0_0);
void setupProcessor();
} // namespace Processor
} // namespace Hardware