#ifndef _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP
#define _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP

#include "DrivingAlgorithm.hpp"
#include "Positioning.hpp"
#include "hw/imu/imu.hpp"

namespace Logic {
namespace DeadReckoning {

class DeadReckoning {
public:
    DeadReckoning();

    void sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg);

    void sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg);

    [[nodiscard]] auto getCurrentPose() const -> Pose2D;

    void setMotorSpeedsHandle(std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds);

private:
    Pose2D currentPosition;

    std::function<DrivingAlgorithm::MotorSpeeds()> getMotorSpeeds;

    struct LinearVelocity2D {
        float dx;
        float dy;
    };

    LinearVelocity2D currentVelocity;
};
} // namespace DeadReckoning
} // namespace Logic

#endif
