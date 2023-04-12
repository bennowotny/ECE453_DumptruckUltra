#include "DeadReckoning.hpp"

namespace Logic {
namespace DeadReckoning {

DeadReckoning::DeadReckoning()
    : currentPosition{.x = 0, .y = 0, .heading = 0},
      currentVelocity{.dx = 0, .dy = 0} {}

void DeadReckoning::sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg) {
    // Accelerometer gives acceleration readings, so it must be integrated twice
    // Expect z-axis to be vertical, so not useful in calculating planar position
    currentVelocity.dx += msg.Ax * msg.Ats;
    currentVelocity.dy += msg.Ay * msg.Ats;

    currentPosition.x += currentVelocity.dx * msg.Ats;
    currentPosition.y += currentVelocity.dy * msg.Ats;
}

void DeadReckoning::sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg) {
    // Gyroscope gives angular velocity readings, so it must be integrated once
    // Expect z-axis to be vertical, so all planar rotation happens about this axis
    currentPosition.heading += msg.Gz * msg.Gts;
}

auto DeadReckoning::getCurrentPose() const -> Pose2D {
    return currentPosition;
}

} // namespace DeadReckoning
} // namespace Logic