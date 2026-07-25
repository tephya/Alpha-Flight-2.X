#ifndef __APP_SHARED_TYPES_H
#define __APP_SHARED_TYPES_H

#include "stm32f4xx_hal.h"

// SystemReadyEventGroup bit定义，ARM解锁前必须全部置位
#define SYSREADY_BIT_HOME_VALID (1 << 0) // Task_Nav置位
#define SYSREADY_BIT_MAG_OK (1 << 1) // Task_Nav置位
#define SYSREADY_BIT_VOLTAGE_OK (1 << 2) // Task_PowerMonit置位
#define SYSREADY_BIT_RC_LINK_OK (1 << 3) // Task_RC_Link置位
#define SYSREADY_BIT_IMU_HEALTH_OK (1 << 4) // Task_FlightCtrl置位，读g_imu_health

#define SYSREADY_ARM_MASK (SYSREADY_BIT_HOME_VALID | SYSREADY_BIT_MAG_OK |     \
                           SYSREADY_BIT_VOLTAGE_OK | SYSREADY_BIT_RC_LINK_OK | \
                           SYSREADY_BIT_IMU_HEALTH_OK)

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
    uint8_t gps_fix_type;           // 0=无定位， 1=GPS， 2=DGPS
    uint8_t gps_satellites;         // 可用卫星数
    uint8_t valid;                  // RMC的A/V标志，‘A’=有效
    int32_t gps_lat, gps_lon;       // GPS经纬度，1e-7度定点整数存储，避免float精度问题
    float gps_altitude_m;           // 海拔，米（GGA）
    float gps_hdop;                 // 水平精度因子（GGA）
    float speed_knots;              // 地速，节（RMC）
    float course;                   // 航向，度（RMC）
    uint8_t home_valid;             // 1=返航点已记录，0=未记录/记录失败
} NavState_t;

typedef struct
{
    uint8_t active_imu_sel;         // 当前该使用哪路数据，0=IMU1,1=IMU2
    uint8_t imu1_healthy;           // 0=异常/1=正常
    uint8_t imu2_healthy;           // 0=异常/1=正常
    uint16_t bad_frame_count;       // 连续异常帧计数
    uint16_t good_frame_count;      // 连续健康帧计数（用于回切判定）
    uint8_t dual_fault;             // 锁存标志，双路都不健康时置1；任意一路健康时由RecordGood清0
} ImuHealthStatus_t;

typedef struct
{
    volatile uint32_t filghtctrl_last_tick;
    // res task
} SystemHeartbeat_t;


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

typedef enum
{
    NAV_CMD_SET_HOME = 0,
} NavCommand_t;

typedef enum
{
    SENSOR_OK = 0,
    SENSOR_FAIL
} SensorStatus_t;

extern volatile ImuHealthStatus_t g_imu_health;
extern volatile SystemHeartbeat_t g_heartbeat;

#endif
