#include "DrivingAlgorithm.hpp"
#include "cy_utils.h"
#include "logic/driving-algorithm/DrivingAlgorithm.hpp"
#include <utility>

namespace Logic {
namespace DrivingAlgorithm {

DrivingAlgorithm::DrivingAlgorithm(DriveMotorLayout driveMotors,
                                   std::function<float()> getDistanceFunction,
                                   std::function<DeadReckoning::Pose2D()> getPoseFunction)
    : leftMotor{driveMotors.leftMotor},
      rightMotor{driveMotors.rightMotor},
      getDistanceFunction{std::move(getDistanceFunction)},
      getPoseFunction{std::move(getPoseFunction)},
      currentTarget{.x = 0, .y = 0, .heading = 0},
      drivingTaskHandle{},
      taskEnabled{false} {
    // Need valid functions
    CY_ASSERT(this->getDistanceFunction != nullptr);
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

} // namespace DrivingAlgorithm
} // namespace Logic