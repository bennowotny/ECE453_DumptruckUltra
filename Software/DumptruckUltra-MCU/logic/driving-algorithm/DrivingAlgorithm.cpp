#include "DrivingAlgorithm.hpp"
#include "Motor.hpp"
#include "cy_utils.h"
#include "logic/dead-reckoning/DeadReckoning.hpp"
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
      drivingTaskHandle{},
      taskEnabled{false} {
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
    leftMotor.disable();
    rightMotor.disable();
    currentStatus = stopStatus;
    // Manipulate task LAST to avoid scheduling shennanigans
    vTaskSuspend(drivingTaskHandle);
}

void DrivingAlgorithm::stop() {
    stop(DrivingAlgorithmStatus::STOPPED);
}

void DrivingAlgorithm::drivingTask() {

    // No setup

    while (true) {
        const auto currPose{getPoseFunction()};
        // If we are not at the target...
        if (distanceToTarget(currPose) > TARGET_PROXIMITY_THRESHOLD_METERS) {
            // Find target heading
            const float targetHeading{std::atan2(currentTarget.y - currPose.y, currentTarget.x - currPose.x)};
            // If something is in the way...
            if (getFrontDistanceFunction() < DISTANCE_THRESHOLD_METERS) {
                // Turn right
                leftMotor.setPower(Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS);
                rightMotor.setPower(-Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS);
            } else {
                // Otherwise, drive at the target
                const auto motorPowers{deltaAngleToDrivePowers(targetHeading - currPose.heading)};
                leftMotor.setPower(motorPowers.leftSpeed);
                rightMotor.setPower(motorPowers.rightSpeed);
            }
        } else {
            // We are at our current target, so stop, signal complete, and wait for restart
            stop(DrivingAlgorithmStatus::COMPLETE);
        }
        // Req'd to prevent constantly resetting the PWM and destroying the waveform
        vTaskDelay(DRIVING_ALGORITHM_TASK_PERIOD_TICKS);
    }
}

// Different methods are here: https://www.desmos.com/calculator/odrruerlc5
auto DrivingAlgorithm::deltaAngleToDrivePowers(float angleDiff) -> DrivingAlgorithm::DrivingPower {
    float leftSpeed{0};
    float rightSpeed{0};
    constexpr float CLIMB_RATE{4};
    if (angleDiff < 0) {
        leftSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS * std::max(static_cast<float>(CLIMB_RATE / M_PI * angleDiff + 1), -1.0F);
        rightSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS;
    } else {
        leftSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS;
        rightSpeed = Hardware::Motors::Motor::MOTOR_MAX_SPEED_ABS * std::max(static_cast<float>(-CLIMB_RATE / M_PI * angleDiff + 1), -1.0F);
    }
    return {.leftSpeed = leftSpeed, .rightSpeed = rightSpeed};
}

auto DrivingAlgorithm::distanceToTarget(const DeadReckoning::Pose2D &currPosition) const -> float {
    // Assume Euclidean (planar) distance
    constexpr float SQUARE_POWER{2};
    return std::sqrt(std::pow(currPosition.x - currentTarget.x, SQUARE_POWER) + std::pow(currPosition.y - currentTarget.y, SQUARE_POWER));
}

} // namespace DrivingAlgorithm
} // namespace Logic