#include "ObjectDetector.hpp"
#include "DeadReckoning.hpp"
#include "vision/ObjectDetector.hpp"

namespace Logic {
namespace Vision {

auto ObjectDetector::detectedObject() const -> bool {
    return false;
}

auto ObjectDetector::currentObjectLocation() const -> DeadReckoning::Pose2D {
    return {};
}

} // namespace Vision
} // namespace Logic
