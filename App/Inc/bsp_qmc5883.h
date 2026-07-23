#ifndef __BSP_QMC5883_H
#define __BSP_QMC5883_H

#include "i2c.h"

typedef struct
{
    int16_t rmx, rmy, rmz;
    float MX, MY, MZ;
} MAG_Data_t;

HAL_StatusTypeDef QMC_Init(void);
static void QMC_Raw2Gauss();
HAL_StatusTypeDef QMC_ReadData(void);
void QMC_CopyTo(MAG_Data_t *out);

#endif
