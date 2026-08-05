#ifndef __APP_IMU2_REDUNDANCY_H
#define __APP_IMU2_REDUNDANCY_H

#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IMU_UPDATE_ACTIVE_FRAME = 0,
    IMU_UPDATE_STANDBY_ONLY,
    IMU_UPDATE_CALIBRATION,
    IMU_UPDATE_TIMEOUT,
    IMU_UPDATE_DUAL_FAULT,
    IMU_UPDATE_TIMING_ANOMALY,
} ImuUpdateResult_t;

ImuUpdateResult_t ImuRedundancy_Update(IcmData_t *out, float *dt_s, uint8_t *fresh_flags);
void ImuRedundancy_Init(uint8_t init_fail_mask);

#endif
