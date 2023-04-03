#ifndef _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP
#define _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP

#include "hw/motors/Motor.hpp"
#include "logic/dead-reckoning/DeadReckoning.hpp"
#include <functional>

namespace Logic {
namespace DrivingAlgorithm {

enum class DrivingAlgorithmStatus {
    STOPPED,
    RUNNING,
    COMPLETE,
    ERROR
};

class DrivingAlgorithm {
public:
    DrivingAlgorithm(const Hardware::Motors::Motor &leftMotor,
                     const Hardware::Motors::Motor &rightMotor,
                     std::function<float()> getDistanceFunction,
                     std::function<DeadReckoning::Pose2D()> getPoseFunction);

    void loadNewTarget(DeadReckoning::Pose2D position);
    void start();
    void stop();
    [[nodiscard]] auto getStatus() const -> DrivingAlgorithmStatus;

private:
    Hardware::Motors::Motor leftMotor;
    Hardware::Motors::Motor rightMotor;
    DeadReckoning::Pose2D currentTarget;

    [[noreturn]] void drivingTask();
};
} // namespace DrivingAlgorithm
} // namespace Logic

#endif