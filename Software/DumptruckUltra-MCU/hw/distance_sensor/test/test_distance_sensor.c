#include "test_distance_sensor.h"

// #include "vl53l1_api.h"
// #include "vl53l1_platform_init.h"

#include <stdio.h>

void print_ranging_data(
    int i,
    VL53L1_RangingMeasurementData_t RangingMeasurementData) {
    printf("Number of measurements = %d\n", i);

    printf("%d: Stream Count= %d\n",
           i, RangingMeasurementData.StreamCount);
    printf("%d: SignalRateRtnMegaCps= %f\n",
           i, RangingMeasurementData.SignalRateRtnMegaCps / 65536.0);
    printf("%d: AmbientRateRtnMegaCps= %f\n",
           i, RangingMeasurementData.AmbientRateRtnMegaCps / 65536.0);
    printf("%d: EffectiveSpadRtnCount= %d\n",
           i, RangingMeasurementData.EffectiveSpadRtnCount);
    printf("%d: SigmaMilliMeter= %f\n",
           i, RangingMeasurementData.SigmaMilliMeter / 65536.0);
    printf("%d: RangeMilliMeter= %d\n",
           i, RangingMeasurementData.RangeMilliMeter);
    printf("%d: RangeStatus= %d\n",
           i, RangingMeasurementData.RangeStatus);

    printf("\n");
}

VL53L1_Error test_distance_sensor(VL53L1_DEV Dev) {

    VL53L1_Error Status = VL53L1_ERROR_NONE;
    // VL53L1_Dev_t dev;
    // VL53L1_DEV Dev = &dev;
    VL53L1_PresetModes PresetMode;
    // VL53L1_DeviceInfo_t DeviceInfo;
    // VL53L1_Version_t Version;
    // VL53L1_ll_version_t llVersion;
    // VL53L1_CalibrationData_t CalData;

    /*
     * Initialize the platform interface
     */
    if (Status == VL53L1_ERROR_NONE)
        Status = VL53L1_platform_init(
            Dev,
            (0x29 << 1), /* EVK requires 8-bit I2C */
            1,           /* comms_type  I2C*/
            400);        /* comms_speed_khz - 400kHz recommended */

    /*
     *  Wait for firmware to finish booting
     */
    if (Status == VL53L1_ERROR_NONE)
        Status = VL53L1_WaitDeviceBooted(Dev);

    /*
     * Initialise Dev data structure
     */
    if (Status == VL53L1_ERROR_NONE)
        Status = VL53L1_DataInit(Dev);

    if (Status == VL53L1_ERROR_NONE)
        Status = VL53L1_StaticInit(Dev);

    if (Status == VL53L1_ERROR_NONE) {
        PresetMode = VL53L1_PRESETMODE_RANGING;
        Status = VL53L1_SetPresetMode(Dev, PresetMode);
    }

    if (Status == VL53L1_ERROR_NONE) {
        printf("*********************************************\n");
        printf("    RUN RunRangingLoop1\n");
        printf("*********************************************\n");
        Status = ranging_loop(Dev, 15);
    }
}

VL53L1_Error ranging_loop(VL53L1_DEV Dev, int no_of_measurements) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t i;
    VL53L1_RangingMeasurementData_t RangingMeasurementData;

    if (Status == VL53L1_ERROR_NONE) {
        printf("run VL53L1_StartMeasurement\n");
        Status = VL53L1_StartMeasurement(Dev);
    }
    if (Status != VL53L1_ERROR_NONE) {
        printf("fail to StartMeasurement\n");
        return -1;
    }

    for (i = 0; i < no_of_measurements; i++) {

        /* Wait for range completion */
        if (Status == VL53L1_ERROR_NONE)
            Status = VL53L1_WaitMeasurementDataReady(Dev);

        if (Status == VL53L1_ERROR_NONE) {
            Status = VL53L1_GetRangingMeasurementData(Dev, &RangingMeasurementData);
            if (Status == VL53L1_ERROR_NONE)
                VL53L1_ClearInterruptAndStartMeasurement(Dev);

            print_ranging_data(i, RangingMeasurementData);
        } else {
            break;
        }
    }

    if (Status == VL53L1_ERROR_NONE) {
        printf("run VL53L1_StopMeasurement\n");
        Status = VL53L1_StopMeasurement(Dev);
    }

    return Status;
}
