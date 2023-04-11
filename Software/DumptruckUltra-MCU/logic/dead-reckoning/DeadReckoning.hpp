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
    void sendAccelerometerMessage(const Hardware::IMU::AccelerometerData &msg) {
        // Do integration
    }

    void sendGyroscopeMessage(const Hardware::IMU::GyroscopeData &msg) {
        // Do integration
    }

    [[nodiscard]] auto getCurrentPose() const -> Pose2D;
};
} // namespace DeadReckoning
} // namespace Logic

#endif
