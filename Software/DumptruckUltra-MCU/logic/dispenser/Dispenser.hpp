#ifndef LOGIC_DISPENSER_DISPENSER_HPP
#define LOGIC_DISPENSER_DISPENSER_HPP

#include "Servo.hpp"

namespace Logic {
namespace Dispenser {
class Dispenser {
public:
    explicit Dispenser(Hardware::Servos::Servo dispenserServo);
    void open();
    void close();
    void placeObject();
    [[nodiscard]] auto getNumObjects() const -> uint32_t;
    [[nodiscard]] auto full() const -> bool;
    void clear();

private:
    Hardware::Servos::Servo dispenserServo;
    uint32_t objectCount;

    static constexpr float OPEN_POSITION{90};
    static constexpr float CLOSE_POSITION{0};
    static constexpr uint32_t MAX_OBJECTS{2};
};
} // namespace Dispenser
} // namespace Logic

#endif