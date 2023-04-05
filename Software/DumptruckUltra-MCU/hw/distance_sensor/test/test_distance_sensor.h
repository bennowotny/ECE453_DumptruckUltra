#ifndef _TEST_DISTANCE_SENSOR_H_
#define _TEST_DISTANCE_SENSOR_H_

// #include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "vl53l1_platform_init.h"

VL53L1_Error test_distance_sensor(VL53L1_DEV Dev);

VL53L1_Error ranging_loop(VL53L1_DEV Dev, int no_of_measurements);

#endif
