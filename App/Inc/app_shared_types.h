#ifndef __APP_SHARED_TYPES_H
#define __APP_SHARED_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// SystemReadyEventGroup bit定义，ARM解锁前必须全部置位
#define SYSREADY_BIT_HOME_VALID (1 << 0) // Task_Nav置位
#define SYSREADY_BIT_MAG_OK (1 << 1) // Task_Nav置位
#define SYSREADY_BIT_VOLTAGE_OK (1 << 2) // Task_PowerMonit置位
#define SYSREADY_BIT_RC_LINK_OK (1 << 3) // Task_RC_Link置位
#define SYSREADY_BIT_IMU_HEALTH_OK (1 << 4) // Task_FlightCtrl置位，读g_imu_health
#define SYSREADY_BIT_RC_CALIB_OK (1 << 5) // Task_RC_link置位
#define SYSREADY_BIT_GYRO_CALIB_OK (1 << 6) // Task_FlightCtrl置位

#define SYSREADY_ARM_MASK (SYSREADY_BIT_MAG_OK |     	\
                           SYSREADY_BIT_VOLTAGE_OK | SYSREADY_BIT_RC_LINK_OK | 	\
                           SYSREADY_BIT_IMU_HEALTH_OK |                       	\
                           SYSREADY_BIT_RC_CALIB_OK |                         	\
                           SYSREADY_BIT_GYRO_CALIB_OK)							

typedef __packed struct
{
    uint16_t time_ms;               // 时间戳
    int16_t angle_cdeg[3];          // 欧拉角
    uint16_t motor[4];              // 电机驱动值
    int8_t target_cdeg[3];          // 目标角
} Blackbox_Frame_t;

typedef enum
{
    NAV_GPS_VELOCITY_SOURCE_RMC = 0U,
    NAV_GPS_VELOCITY_SOURCE_POSITION_WINDOW = 1U,
} NavGpsVelocitySource_t;

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

    float gps_velocity_n_mps;
    float gps_velocity_e_mps;

    /* 两种候选观测，供选择逻辑和Blackbox诊断使用。 */
    float rmc_velocity_n_mps;
    float rmc_velocity_e_mps;
    float gps_position_velocity_n_mps;
    float gps_position_velocity_e_mps;

    uint32_t rmc_sequence;          // 每解析到一条checksum正确的RMC递增
    uint32_t rmc_last_update_ms;
    uint16_t rmc_period_ms;
    uint16_t rmc_age_ms;

    uint8_t rmc_velocity_valid;         // RMC speed/course样本质量有效
    uint8_t gps_position_velocity_valid;    // Position窗口Velocity有效
    uint8_t gps_velocity_valid;         // 当前选中观测有效
    uint8_t gps_velocity_source;
    uint8_t gps_velocity_control_ready;     // 更新率/年龄满足闭环控制要求
    uint8_t gps_position_control_ready;     // 经纬度、更新率和水平定位质量满足Position Hold要求
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
    volatile uint32_t flightctrl_last_tick;
    volatile uint32_t powermonit_last_tick;
    // res task
} SystemHeartbeat_t;

typedef struct
{
    bool voltage_fault;     // true=持续低压已确定，供Arm_Update做紧急disarm判据
    
    bool current_limiting;      // 0=未触发限流保护，1=正在压低油门
    uint16_t current_limit_permille;    // 油门缩放比例，600~1000对应60%~100%
    float current_filtered_a;           // 滤波后的整机总电流，仅用于监视和记录
} PowerHealth_t;


typedef enum
{
    ARM_STATE_DISARMED = 0,
    ARM_STATE_ARMED = 1,
} ArmState_t;

typedef enum
{
    EVT_ARMED = 0,               // 解锁
    EVT_DISARMED = 1,            // 未解锁
    EVT_LOW_BATTERY = 2,         // 低电压(<14.0V)
    EVT_CRITICAL_BATTERY = 3,    // 超低电压(<13.2V)，持续报警
    EVT_GPS_FIX_ACQUIRED = 4,    // GPS记录返航点成功
    EVT_SD_CARD_ERROR = 5,       // 读写SD卡出错
    EVT_IMU_FAULT = 6,           // IMU通信错误
    EVT_RC_LOST = 7,             // 遥控信号丢失，持续报警
    EVT_CURRENT_LIMITING = 8,    // 过流保护中
    EVT_RC_CALIB_STARTED = 9,    // RC校准开始
    EVT_RC_CALIB_SUCCESS = 10,   // RC校准成功
    EVT_RC_CALIB_FAILED = 11,    // RC校准失败
    EVT_GYRO_CALIB_SUCCESS = 12, // Gyro校准成功
    EVT_LEVEL_TRIM_STARTED = 13, // Level Trim开始采样
    EVT_LEVEL_TRIM_SUCCESS = 14, // Level Trim保存成功
    EVT_LEVEL_TRIM_FAILED = 15,  // Level Trim失败，旧配置保持不变
    EVT_MAG_CAL_CAPTURE_STARTED = 16,   // 开始Mag校准采样
    EVT_MAG_CAL_CAPTURE_DONE = 17,      // 结束Mag校准采样
    EVT_MAG_CAL_CAPTURE_FAILED = 18,    // Mag校准采样失败
}
IndicatorEvent_t;

typedef enum
{
    NAV_CMD_SET_HOME = 0,
} NavCommand_t;

typedef enum
{
    SENSOR_OK = 0,
    SENSOR_FAIL,
} SensorStatus_t;

extern volatile ImuHealthStatus_t g_imu_health;
extern volatile SystemHeartbeat_t g_heartbeat;
extern volatile ArmState_t g_arm_state;
extern volatile PowerHealth_t g_power_health;

#endif
