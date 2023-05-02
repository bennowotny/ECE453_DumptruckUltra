#include "DrivingAlgorithm.hpp"
#include "Motor.hpp"
#include "cy_utils.h"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
#include "portmacro.h"
#include <cmath>
#include <utility>

namespace Logic {
namespace DrivingAlgorithm {

DrivingAlgorithm::DrivingAlgorithm(DriveMotorLayout driveMotors,
                                   std::function<float()> getFrontDistanceFunction,
                                   std::function<DeadReckoning::Pose2D()> getPoseFunction)
    : leftMotor{driveMotors.leftMotor},
      rightMotor{driveMotors.rightMotor},
      getFrontDistanceFunction{std::move(getFrontDistanceFunction)},
      getPoseFunction{std::move(getPoseFunction)},
      currentTarget{.x = 0, .y = 0, .heading = 0},
      currentStatus{DrivingAlgorithmStatus::STOPPED},
      drivingTaskHandle{} {

    // Need valid functions
    CY_ASSERT(this->getFrontDistanceFunction != nullptr);
    CY_ASSERT(this->getPoseFunction != nullptr);

    // Create FreeRTOS task
    const auto drivingTaskSetupResult{
        xTaskCreate(
            [](void *obj) { static_cast<DrivingAlgorithm *>(obj)->drivingTask(); },
            DRIVING_ALGORITHM_TASK_NAME,
            DRIVING_ALGORITHM_STACK_SIZE,
            this,
            DRIVING_ALGORITHM_PRIORITY,
            &drivingTaskHandle)};
    CY_ASSERT(drivingTaskSetupResult == pdPASS);
}

auto DrivingAlgorithm::getStatus() const -> DrivingAlgorithmStatus {
    return currentStatus;
}

void DrivingAlgorithm::loadNewTarget(const DeadReckoning::Pose2D &position) {
    currentTarget = position;
}

void DrivingAlgorithm::start() {
    leftMotor.enable();
    rightMotor.enable();
    currentStatus = DrivingAlgorithmStatus::RUNNING;
    // Manipulate task LAST to avoid scheduling shennanigans
    vTaskResume(drivingTaskHandle);
}

void DrivingAlgorithm::stop(const DrivingAlgorithmStatus &stopStatus) {
    setPower({.leftPower = 0, .rightPower = 0});
    currentStatus = stopStatus;
    // Manipulate task LAST to avoid scheduling shennanigans
    vTaskSuspend(drivingTaskHandle);
}

void DrivingAlgorithm::stop() {
    stop(DrivingAlgorithmStatus::STOPPED);
}

void DrivingAlgorithm::setPower(MotorSpeeds speeds) {
    if (currentStatus == DrivingAlgorithmStatus::STOPPED) {
        leftMotor.enable();
        rightMotor.enable();
        currentStatus = DrivingAlgorithmStatus::RUNNING;
    }

    leftMotor.setPower(speeds.leftPower);
    rightMotor.setPower(speeds.rightPower);
}

[[nodiscard]] auto DrivingAlgorithm::getPower() const -> MotorSpeeds {
    return {.leftPower = leftMotor.getPower(), .rightPower = rightMotor.getPower()};
}

void DrivingAlgorithm::drivingTask() {

    // No setup

    while (true) {
        auto currPose{getPoseFunction()};

        // If we are not at the target...
        if (distanceToTarget(currPose) > TARGET_PROXIMITY_THRESHOLD_METERS) { //|| std::abs(angleDiff(currPose.heading, currentTarget.heading)) > TARGET_HEADING_THRESHOLD_DEGREES) {
            // Find target heading (if we are already close to the target, turn to match the goal heading)
            const float headingToTarget{(distanceToTarget(currPose) > TARGET_PROXIMITY_THRESHOLD_METERS)
                                            ? std::fmod(static_cast<float>(std::atan2(currentTarget.y - currPose.y, currentTarget.x - currPose.x) * 180.F / M_PI - 90.F + 360.F), 360.F)
                                            : currentTarget.heading};
            // If something is in the way...
            if (getFrontDistanceFunction() < DISTANCE_THRESHOLD_METERS) {
                // Turn right
                setPower(
                    {.leftPower = 0.3,
                     .rightPower = -0.3});
            } else {
                // Otherwise, drive at the target
                const auto motorPowers{deltaAngleToDrivePowers(angleDiff(currPose.heading, headingToTarget))};
                setPower(motorPowers);
            }
        } else {
            // We are at our current target, so stop, signal complete, and wait for restart
            stop(DrivingAlgorithmStatus::COMPLETE);
        }
        // Req'd to prevent constantly resetting the PWM and destroying the waveform
        vTaskDelay(DRIVING_ALGORITHM_TASK_PERIOD_TICKS);
    }
}

/* Different methods are here: https://www.desmos.com/calculator/odrruerlc5
 * In short:
 *  - If the target is to the left, slow down the left wheel and keep the right wheel at full power
 *  - Otherwise, slow down the right wheel and keep the left wheel at full power
 * This change happens linearly with both wheels at maximum power at:
 *  - 0 (driving forward)
 *  - pi/2 (turning right)
 *  - pi/2 (turning left)
 */
auto DrivingAlgorithm::deltaAngleToDrivePowers(float angleDiff) -> MotorSpeeds {
    float leftSpeed{0};
    float rightSpeed{0};
    angleDiff = angleDiff > 180.F ? angleDiff - 360.F : angleDiff;
    constexpr float CLIMB_RATE{2 / 135.0};
    if (angleDiff < 0) {
        leftSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS * std::max(static_cast<float>(CLIMB_RATE * angleDiff + 1), -1.0F) / 1.3;
        rightSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS / 1.3;
    } else {
        leftSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS / 1.3;
        rightSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS * std::max(static_cast<float>(-CLIMB_RATE * angleDiff + 1), -1.0F) / 1.3;
    }
    return {.leftPower = leftSpeed, .rightPower = rightSpeed};
}

auto DrivingAlgorithm::distanceToTarget(const DeadReckoning::Pose2D &currPosition) const -> float {
    // Assume Euclidean (planar) distance
    constexpr float SQUARE_POWER{2};
    return std::sqrt(std::pow(currPosition.x - currentTarget.x, SQUARE_POWER) + std::pow(currPosition.y - currentTarget.y, SQUARE_POWER));
}

auto DrivingAlgorithm::angleDiff(float heading1, float heading2) -> float {
    const auto diff1{std::fmod(heading1 - heading2 + 360.F, 360.F)};
    const auto diff2{std::fmod(heading2 - heading1 + 360.F, 360.F)};

    const auto usedDiff{std::abs(diff1) > std::abs(diff2) ? diff2 : diff1};

    return usedDiff > 180.F ? usedDiff - 360.F : usedDiff;
}

} // namespace DrivingAlgorithm
} // namespace Logic