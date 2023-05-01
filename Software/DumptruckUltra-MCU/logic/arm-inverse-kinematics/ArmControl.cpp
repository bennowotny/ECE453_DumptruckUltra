#include "ArmControl.hpp"

namespace Logic {
namespace Arm {

using Hardware::Servos::Servo;

ArmControl::ArmControl(const ArmLayout &armLayout, std::function<bool()> isClawOpen) {
}

void ArmControl::collect(double distanceForward_m) {
}

} // namespace Arm
} // namespace Logic
