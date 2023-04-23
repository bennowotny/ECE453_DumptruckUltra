#ifndef _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP
#define _LOGIC_DEAD_RECKONING_DEADRECKONING_HPP

#include "hw/imu/imu.hpp"

namespace Logic {
namespace DeadReckoning {

struct Pose2D {
    float x;
    float y;
    float heading;
};

class DeadReckoning {
public:
    DeadReckoning();

    void sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg);

    void sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg);

    [[nodiscard]] auto getCurrentPose() const -> Pose2D;

private:
    Pose2D currentPosition;

    struct LinearVelocity2D {
        float dx;
        float dy;
    };

    LinearVelocity2D currentVelocity;
};
} // namespace DeadReckoning
} // namespace Logic

#endif
