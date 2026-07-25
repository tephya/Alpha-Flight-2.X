#ifndef __APP_IMU2_REDUNDANCY_H
#define __APP_IMU2_REDUNDANCY_H

#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

bool ImuRedundancy_Update(IcmData_t *out, float *dt_s);
void ImuRedundancy_Init(void);

#endif
