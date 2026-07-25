#ifndef __BSP_QMC5883_H
#define __BSP_QMC5883_H

#include "i2c.h"

typedef struct
{
    float MX, MY, MZ; // 已转换Gauss值，已对齐NED坐标系
} MagData_t;

HAL_StatusTypeDef QMC_Init(void);
static void QMC_Raw2Gauss(void);
HAL_StatusTypeDef QMC_ReadData(void);
void QMC_CopyTo(MagData_t *out);

#endif
