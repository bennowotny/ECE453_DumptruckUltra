#ifndef _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP
#define _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP

#include "FreeRTOS.h" // IWYU pragma: keep
#include "hw/motors/Motor.hpp"
#include "logic/dead-reckoning/DeadReckoning.hpp"
#include "task.h" // IWYU pragma: keep
#include <functional>

namespace Logic {
namespace DrivingAlgorithm {

enum class DrivingAlgorithmStatus {
    STOPPED,
    RUNNING,
    COMPLETE,
    ERROR
};

struct DriveMotorLayout {
    Hardware::Motors::Motor leftMotor;
    Hardware::Motors::Motor rightMotor;
};

class DrivingAlgorithm {
public:
    DrivingAlgorithm(DriveMotorLayout driveMotors,
                     std::function<float()> getFrontDistanceFunction,
                     std::function<DeadReckoning::Pose2D()> getPoseFunction);

    void loadNewTarget(DeadReckoning::Pose2D position);
    void start();
    void stop();
    [[nodiscard]] auto getStatus() const -> DrivingAlgorithmStatus;

private:
    Hardware::Motors::Motor leftMotor;
    Hardware::Motors::Motor rightMotor;
    const std::function<float()> getFrontDistanceFunction;
    const std::function<DeadReckoning::Pose2D()> getPoseFunction;
    DeadReckoning::Pose2D currentTarget;

    TaskHandle_t drivingTaskHandle;
    bool taskEnabled;

    [[noreturn]] void drivingTask();

    struct DrivingPower {
        float leftPower;
        float rightPower;
    };

    [[nodiscard]] auto deltaAngleToDrivePowers(float angleDiff) -> DrivingPower;

    static constexpr auto DRIVING_ALGORITHM_TASK_NAME{"Driving Algorithm"};
    static constexpr uint16_t DRIVING_ALGORITHM_STACK_SIZE{configMINIMAL_STACK_SIZE};
    static constexpr uint32_t DRIVING_ALGORITHM_PRIORITY{tskIDLE_PRIORITY + 1};
};
} // namespace DrivingAlgorithm
} // namespace Logic

#endif