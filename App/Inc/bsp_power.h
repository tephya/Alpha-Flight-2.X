#ifndef __BSP_POWER_H
#define __BSP_POWER_H

#include <stdint.h>

typedef struct
{
    float vbat;         // 电池电压，单位V
    float current;      /*电调电流检测输出，单位A。当前无使用方，读出仅做保留，
                            供以后做过流保护时用，不在本次判断范围内*/
} PowerData_t;

void BSP_Power_Init(void);
void BSP_Power_Read(PowerData_t *out);

#endif
