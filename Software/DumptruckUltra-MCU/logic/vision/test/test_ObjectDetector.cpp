#include "logic/vision/ObjectDetector.hpp"
#include "cyhal_system.h"

int main(void){
    Logic::Vision::ObjectDetector vision;
    while (true) {
        if (vision.detectedObject()) {
            auto pose = vision.currentObjectLocation();
            printf("x: %f, y: %f\n", pose.x, pose.y);
        }
        cyhal_system_delay_ms(100);
    }
}