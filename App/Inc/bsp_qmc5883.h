#ifndef __BSP_QMC5883_H
#define __BSP_QMC5883_H

#include "i2c.h"
#include "stdbool.h"

typedef struct
{
    float MX, MY, MZ;   // 已转换Gauss值，已对齐NED坐标系
    bool ovfl;          // true=任一一轴数据溢出(超过±30000 LSB)，本次数据不可信
} MagData_t;

HAL_StatusTypeDef QMC_Init(void);
HAL_StatusTypeDef QMC_ReadData(void);
void QMC_CopyTo(MagData_t *out);

#endif
