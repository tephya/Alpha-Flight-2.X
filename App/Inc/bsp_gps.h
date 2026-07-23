#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef struct
{
    uint8_t gps_fix_type;       // 0=无定位， 1=GPS， 2=DGPS
    uint8_t gps_satellites;     // 可用卫星数
    uint8_t valid;              // RMC的A/V标志，‘A’=有效

    int32_t gps_lat, gps_lon;   // GPS经纬度，1e-7度定点整数存储，避免float精度问题

    float gps_altitude_m;   // 海拔，米（GGA）
    float gps_hdop;         // 水平精度因子（GGA）

    float speed_knots;     // 地速，节（RMC）
    float course;          // 航向，度（RMC）
} GPS_Data_t;

typedef struct
{
    int32_t home_lat, home_lon; // 起飞点的经纬度
    float altitude;             // 起飞点的海拔高度
} GPS_Home_t;

void GPS_Init(void);
void GPS_Poll(void);
bool GPS_SetHome(void);
void GPS_CopyDataTo(GPS_Data_t *out);
void GPS_CopyHomeTo(GPS_Home_t *out);

#endif
