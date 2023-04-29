#ifndef _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP
#define _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP

#include "DrivingAlgorithm.hpp"
#include "Positioning.hpp"
#include "hw/imu/imu.hpp"

namespace Logic {
namespace DeadReckoning {

class DeadReckoning {
public:
    explicit DeadReckoning(std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds);

    void sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg);

    void sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg);

    [[nodiscard]] auto getCurrentPose() const -> Pose2D;

private:
    Pose2D currentPosition;

    const std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds;

    struct LinearVelocity2D {
        float dx;
        float dy;
    };

    LinearVelocity2D currentVelocity;
};
} // namespace DeadReckoning
} // namespace Logic

#endif
