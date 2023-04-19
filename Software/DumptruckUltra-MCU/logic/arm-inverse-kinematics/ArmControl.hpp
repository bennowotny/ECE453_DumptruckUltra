#ifndef LOGIC_ARM_INVERSE_KINEMATICS_ARMCONTROL_HPP
#define LOGIC_ARM_INVERSE_KINEMATICS_ARMCONTROL_HPP

#include "Servo.hpp"
#include <functional>

namespace Logic {
namespace Arm {

using Hardware::Servos::Servo;

struct ArmLayout {
    Servo shoulder;
    Servo elbow;
    Servo wrist;
    Servo claw;
};

class ArmControl {
public:
    ArmControl(const ArmLayout &armLayout, std::function<bool()> isClawOpen);

    void collect(double distanceForward_m);

private:
};
} // namespace Arm
} // namespace Logic

#endif