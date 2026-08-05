#ifndef __APP_RC_CALIBRATION_H
#define __APP_RC_CALIBRATION_H

#include "bsp_elrs.h"
#include <stdbool.h>
#include <stdint.h>

#define RC_CAL_CHANNEL_COUNT 4U

typedef struct
{
    uint16_t min[RC_CAL_CHANNEL_COUNT];
    uint16_t mid[RC_CAL_CHANNEL_COUNT];
    uint16_t max[RC_CAL_CHANNEL_COUNT];
    uint16_t deadband;
    uint16_t reserved;
} RcCalibration_t;

/* 读取Flash中最新校准；无有效记录时加载CRSF默认值但保持not-ready。*/
void RcCalibration_Init(void);

/* 使用尚未归一化的原始CRSF通道推进校准状态机。 */
void RcCalibration_Update(const RCChannelData_t *raw_rc);

/* 将channels[0..3]原始值映射到标准CRSF 172..1811范围。 */
void RcCalibration_Apply(RCChannelData_t *rc);

bool RcCalibration_IsReady(void);
bool RcCalibration_IsActive(void);
uint16_t RcCalibration_GetNormalizedDeadband(void);

#endif
