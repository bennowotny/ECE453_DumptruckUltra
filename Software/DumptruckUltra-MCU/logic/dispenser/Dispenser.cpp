#include "Dispenser.hpp"

namespace Logic {
namespace Dispenser {
Dispenser::Dispenser(Hardware::Servos::Servo dispenserServo)
    : dispenserServo{dispenserServo},
      objectCount{0} {
    dispenserServo.enable();
    close();
}

void Dispenser::open() {
    dispenserServo.setPosition(OPEN_POSITION);
}

void Dispenser::close() {
    dispenserServo.setPosition(CLOSE_POSITION);
}

void Dispenser::placeObject() {
    ++objectCount;
}

[[nodiscard]] auto Dispenser::getNumObjects() const -> uint32_t {
    return objectCount;
}

[[nodiscard]] auto Dispenser::full() const -> bool {
    return getNumObjects() >= MAX_OBJECTS;
}

void Dispenser::clear() {
    objectCount = 0;
}

} // namespace Dispenser
} // namespace Logic
