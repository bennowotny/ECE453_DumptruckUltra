#include "DrivingAlgorithm.hpp"
#include "cy_utils.h"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
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

void DrivingAlgorithm::start() {
    vTaskResume(drivingTaskHandle);
    leftMotor.enable();
    rightMotor.enable();
}

void DrivingAlgorithm::stop() {
    vTaskSuspend(drivingTaskHandle);
    leftMotor.disable();
    rightMotor.disable();
}

void DrivingAlgorithm::drivingTask() {

    // Setup?

    while (true) {
        // Find target heading
        const auto currPose{getPoseFunction()};
        const float targetHeading{std::atan2(currentTarget.y - currPose.y, currentTarget.x - currPose.x)};
        if (getFrontDistanceFunction() < 0) { // FIXME: Create threshold (static constexpr, probably)
            // Turn right
            leftMotor.setPower(100);
            rightMotor.setPower(0);
        } else {
            const auto motorPowers{deltaAngleToDrivePowers(targetHeading - currPose.heading)};
            leftMotor.setPower(motorPowers.leftPower);
            rightMotor.setPower(motorPowers.rightPower);
        }
    }
}

// Different methods are here: https://www.desmos.com/calculator/ma2ul0p4ae
auto DrivingAlgorithm::deltaAngleToDrivePowers(float angleDiff) -> DrivingAlgorithm::DrivingPower {
    if (angleDiff < 0 && angleDiff > -M_PI_2) {
        return {.leftPower = std::max((float)(4 / M_PI * angleDiff + 1), -1.0F), .rightPower = 1};
    }
    elif x<-180 : return DriveTrainCmd(left_value = 1, right_value = max(-2 / 90 * x - 7, -1))
               elif x> 0 and
        x < 180 : return DriveTrainCmd(left_value = 1, right_value = max(-2 / 90 * x + 1, -1)) else : #X is greater than 180 return DriveTrainCmd(left_value = max(2 / 90 * x - 7, -1), right_value = 1)
}

} // namespace DrivingAlgorithm
} // namespace Logic