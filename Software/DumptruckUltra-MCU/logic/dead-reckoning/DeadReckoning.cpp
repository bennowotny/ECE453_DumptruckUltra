#include "DeadReckoning.hpp"

#include "DrivingAlgorithm.hpp"
#include <cmath>
#include <utility>

namespace Logic {
namespace DeadReckoning {

DeadReckoning::DeadReckoning()
    : currentPosition{.x = 0, .y = 0, .heading = 0},
      getMotorSpeeds{[]() { return DrivingAlgorithm::MotorSpeeds{.leftPower = 0, .rightPower = 0}; }},
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
    currentVelocity.dx += (msg.Ax * msg.Ats) * std::cos(M_PI / 180.F * currentPosition.heading) + (msg.Ay * msg.Ats) * std::sin(M_PI / 180.F * currentPosition.heading);
    currentVelocity.dy += (msg.Ax * msg.Ats) * std::cos(M_PI / 180.F * currentPosition.heading) - (msg.Ay * msg.Ats) * std::sin(M_PI / 180.F * currentPosition.heading);

    currentPosition.x += (currentVelocity.dx * msg.Ats);
    currentPosition.y += (currentVelocity.dy * msg.Ats);
}

void DeadReckoning::sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg) {
    // Gyroscope gives angular velocity readings, so it must be integrated once
    // Expect z-axis to be vertical, so all planar rotation happens about this axis
    currentPosition.heading += (msg.Gz * msg.Gts);
    currentPosition.heading = std::fmod(currentPosition.heading, 360.F);
}

auto DeadReckoning::getCurrentPose() const -> Pose2D {
    return currentPosition;
}

void DeadReckoning::setMotorSpeedsHandle(std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds) {
    this->getMotorSpeeds = std::move(getMotorSpeeds);
}

} // namespace DeadReckoning
} // namespace Logic