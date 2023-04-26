#ifndef LOGIC_VISION_OBJECTDETECTOR_HPP
#define LOGIC_VISION_OBJECTDETECTOR_HPP

#include "DeadReckoning.hpp"

namespace Logic {
namespace Vision {

class ObjectDetector {
public:
    [[nodiscard]] auto detectedObject() const -> bool;
    [[nodiscard]] auto currentObjectLocation() const -> DeadReckoning::Pose2D; // WORLD FRAME
};

} // namespace Vision
} // namespace Logic

#endif