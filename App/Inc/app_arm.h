#ifndef __APP_ARM_H
#define __APP_ARM_H

#include "app_shared_types.h"
#include "bsp_elrs.h"

void Arm_Init(void);
void Arm_Update(const RCChannelData_t *rc, float roll_meas, float pitch_meas);

#endif
