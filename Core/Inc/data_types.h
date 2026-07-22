#ifndef __DATA_TYPES_H
#define __DATA_TYPES_H

#include "stm32f4xx_hal.h"

typedef __packed struct
{
    uint16_t time_ms;               // 时间戳
    int16_t angle_cdeg[3];          // 欧拉角
    uint16_t motor[4];              // 电机驱动值
    int8_t target_cdeg[3];          // 目标角
} Blackbox_Frame_t;

typedef struct
{
    uint16_t channels[16];          // 16路遥控通道值(172~1811)
    uint8_t link_quality;           // ELRS链路信号质量(LQ)(0~100)
    uint8_t rssi;                   // 信号强度
} RCChannelData_t;

typedef struct
{
    uint8_t fix;            // 0=无定位， 1=GPS， 2=DGPS
    uint8_t satellites;     // 可用卫星数
    uint8_t valid;          // RMC的A/V标志，‘A’=有效

    int32_t latitude;         // 十进制-度，北正南负，纬度
    int32_t longitude;        // 十进制-度，东正西负，经度

    float altitude;         // 海拔，米（GGA）
    float hdop;             // 水平精度因子（GGA）

    float speed_knots;      // 地速，节（RMC）
    float course;           // 航向，度（RMC）
} GPS_Data_t;

typedef struct
{
    float latitude;         // home 点的纬度
    float longitude;        // home 点的经度
    float altitude;         // home 点的 海拔高度
    uint8_t valid;          // home 点是否已记录; 1=已记录， 0=未记录
} GPS_Home_t;

typedef enum
{
    EVT_ARMED,                      // 解锁
    EVT_DISARMED,                   // 未解锁
    EVT_LOW_BATTERY,                // 低电压(<14.0V)
    EVT_CRITICAL_BATTERY,           // 超低电压(<13.2V)
    EVT_GPS_FIX_ACQUIRED,           
    EVT_SD_CARD_FULL,               
    EVT_SD_CARD_ERROR,              // 读写SD卡出错
    EVT_IMU_FAULT,                  // IMU通信错误
    EVT_RC_LOST,                    // 遥控信号丢失
} IndicatorEvent_t;

#endif
