#ifndef _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP
#define _LOGIC_DRIVING_ALGORITHM_DRIVINGALGORITHM_HPP

#include "FreeRTOS.h" // IWYU pragma: keep
#include "hw/motors/Motor.hpp"
#include "logic/dead-reckoning/Positioning.hpp"
#include "portmacro.h"
#include "projdefs.h"
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

struct MotorSpeeds {
    float leftPower;
    float rightPower;
};

class DrivingAlgorithm {
public:
    DrivingAlgorithm(DriveMotorLayout driveMotors,
                     std::function<float()> getFrontDistanceFunction,
                     std::function<DeadReckoning::Pose2D()> getPoseFunction);

    void loadNewTarget(const DeadReckoning::Pose2D &position);
    void start();
    void stop();
    [[nodiscard]] auto getStatus() const -> DrivingAlgorithmStatus;

    void setPower(MotorSpeeds speeds);
    [[nodiscard]] auto getPower() const -> MotorSpeeds;

private:
    Hardware::Motors::Motor leftMotor;
    Hardware::Motors::Motor rightMotor;
    const std::function<float()> getFrontDistanceFunction;
    const std::function<DeadReckoning::Pose2D()> getPoseFunction;
    DeadReckoning::Pose2D currentTarget;
    DrivingAlgorithmStatus currentStatus;

    TaskHandle_t drivingTaskHandle;

    [[noreturn]] void drivingTask();

    [[nodiscard]] auto static deltaAngleToDrivePowers(float angleDiff) -> MotorSpeeds;
    [[nodiscard]] auto distanceToTarget(const DeadReckoning::Pose2D &currPosition) const -> float;
    void stop(const DrivingAlgorithmStatus &stopStatus);

    static constexpr auto DRIVING_ALGORITHM_TASK_NAME{"Driving Algorithm"};
    static constexpr uint16_t DRIVING_ALGORITHM_STACK_SIZE{configMINIMAL_STACK_SIZE * 2};
    static constexpr uint32_t DRIVING_ALGORITHM_PRIORITY{tskIDLE_PRIORITY + 1};
    static constexpr TickType_t DRIVING_ALGORITHM_TASK_PERIOD_TICKS{pdMS_TO_TICKS(2)};

    static constexpr float DISTANCE_THRESHOLD_METERS{0.5};
    static constexpr float TARGET_PROXIMITY_THRESHOLD_METERS{0.1};
    static constexpr float TARGET_HEADING_THRESHOLD_RADIANS{0.1};
};
} // namespace DrivingAlgorithm
} // namespace Logic

#endif