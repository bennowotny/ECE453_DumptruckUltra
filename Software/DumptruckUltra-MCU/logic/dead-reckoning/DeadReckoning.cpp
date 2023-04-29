#include "DeadReckoning.hpp"

#include "DrivingAlgorithm.hpp"
#include <cmath>
#include <utility>

namespace Logic {
namespace DeadReckoning {

DeadReckoning::DeadReckoning(std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds)
    : currentPosition{.x = 0, .y = 0, .heading = 0},
      getMotorSpeeds{std::move(getMotorSpeeds)},
      currentVelocity{.dx = 0, .dy = 0} {}

void DeadReckoning::sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg) {
    // Accelerometer gives acceleration readings, so it must be integrated twice
    // Expect z-axis to be vertical, so not useful in calculating planar position

    const auto speeds{getMotorSpeeds()};
    const auto summedSpeed{speeds.leftPower + speeds.rightPower};

    // Simple 'weighing' based on motor speeds / if we aren't driving, we aren't moving.
    if (std::abs(summedSpeed) < 0.05) {
        currentVelocity = {.dx = 0,
                           .dy = 0};
        return;
    }
    currentVelocity.dx += (msg.Ax * msg.Ats);
    currentVelocity.dy += (msg.Ay * msg.Ats);

    currentPosition.x += (currentVelocity.dx * msg.Ats);
    currentPosition.y += (currentVelocity.dy * msg.Ats);
}

void DeadReckoning::sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg) {
    // Gyroscope gives angular velocity readings, so it must be integrated once
    // Expect z-axis to be vertical, so all planar rotation happens about this axis
    currentPosition.heading += (msg.Gz * msg.Gts);
}

auto DeadReckoning::getCurrentPose() const -> Pose2D {
    return currentPosition;
}

} // namespace DeadReckoning
} // namespace Logic