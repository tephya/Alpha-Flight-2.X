/**
 * @file    app_shared_types.h
 * @brief   应用层共享状态，消息类型及跨任务公共数据定义。
 */

#ifndef __APP_SHARED_TYPES_H
#define __APP_SHARED_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/*
 * SystemReady Event Group 位定义。
 *
 * 各功能 Task 维护自己负责的 Ready Bit；
 * Arm 状态机通过 SYSREADY_ARM_MASK 统一判断解锁前置条件。
 */
#define SYSREADY_BIT_HOME_VALID (1 << 0)    /**< Home 已建立，由 Nav Task 维护。 */
#define SYSREADY_BIT_MAG_OK (1 << 1)        /**< Mag 数据链健康，由 Nav Task 维护。 */
#define SYSREADY_BIT_VOLTAGE_OK (1 << 2)    /**< 电池电压状态正常，由 PowerMonit Task 维护。 */
#define SYSREADY_BIT_RC_LINK_OK (1 << 3)    /**< RC Link 正常，由 RcLink Task 维护。 */
#define SYSREADY_BIT_IMU_HEALTH_OK (1 << 4) /**< 至少存在可用 IMU，由 FlightCtrl Task 维护。 */
#define SYSREADY_BIT_RC_CALIB_OK (1 << 5)   /**< RC Calibration 有效且空闲，由 RcLink Task 维护。 */
#define SYSREADY_BIT_GYRO_CALIB_OK (1 << 6) /**< 启动 Gyro Bias Calibration 完成，由 FlightCtrl Task 维护。 */

/*
 * Arm 所需的全部 System Ready 条件。
 *
 * HOME_VALID 不属于基础解锁条件；
 * GPS Home 尚未建立时仍允许 Manual Flight。
 */
#define SYSREADY_ARM_MASK (SYSREADY_BIT_MAG_OK |     	\
                           SYSREADY_BIT_VOLTAGE_OK | SYSREADY_BIT_RC_LINK_OK | 	\
                           SYSREADY_BIT_IMU_HEALTH_OK |                       	\
                           SYSREADY_BIT_RC_CALIB_OK |                         	\
                           SYSREADY_BIT_GYRO_CALIB_OK)

/**
 * @brief Blackbox 基础控制帧。
 *
 * @note 使用 Packed Layout 写入二进制日志，修改字段时需要同步检查
 *       Blackbox 解析端的数据格式。
 */
typedef __packed struct
{
    uint16_t time_ms;      /**< 时间戳，ms。 */
    int16_t angle_cdeg[3]; /**< Roll/Pitch/Yaw，0.01 deg。 */
    uint16_t motor[4];     /**< 四路 Motor Output。 */
    int8_t target_cdeg[3]; /**< Roll/Pitch/Yaw Target，0.01 deg。 */
} Blackbox_Frame_t;

/**
 * @brief GPS 水平速度观测来源。
 */
typedef enum
{
    NAV_GPS_VELOCITY_SOURCE_RMC = 0U,             /**< RMC Ground Speed + Course。 */
    NAV_GPS_VELOCITY_SOURCE_POSITION_WINDOW = 1U, /**< Position-window 回归速度。 */
} NavGpsVelocitySource_t;

/**
 * @brief Nav Task 发布的最新 GPS Navigation 状态。
 */
typedef struct
{
    uint8_t gps_fix_type;           /**< GPS Fix 类型：0=无定位，1=GPS，2=DGPS。 */
    uint8_t gps_satellites;         /**< 当前可用卫星数量。 */
    uint8_t valid;                  /**< RMC 状态字符：'A'=有效，'V'=无效。 */

    int32_t gps_lat;                /**< 纬度，deg × 1e7。 */
    int32_t gps_lon;                /**< 经度，deg × 1e7。 */
    float gps_altitude_m;           /**< GGA 海拔高度，m。 */
    float gps_hdop;                 /**< GGA Horizontal Dilution of Precision。 */

    float speed_knots;              /**< RMC Ground Speed，knot。 */
    float course;                   /**< RMC Course Over Ground，deg，相对 True North。 */

    uint8_t home_valid;     /**< Home 是否已经成功建立。 */

    float gps_velocity_n_mps;   /**< 当前选定的 North Velocity Observation，m/s。 */
    float gps_velocity_e_mps;   /**< 当前选定的 East Velocity Observation，m/s。 */

    /*
     * 两种 GPS Velocity Candidate。
     *
     * 当前控制观测固定选用 RMC；
     * Position-window Velocity 保留用于一致性检查、Braking 判断与诊断。
     */
    float rmc_velocity_n_mps;          /**< RMC 推导的 North Velocity，m/s。 */
    float rmc_velocity_e_mps;          /**< RMC 推导的 East Velocity，m/s。 */
    float gps_position_velocity_n_mps; /**< Position-window North Velocity，m/s。 */
    float gps_position_velocity_e_mps; /**< Position-window East Velocity，m/s。 */

    uint32_t rmc_sequence;       /**< 每成功解析一条合法 RMC 后递增的序号。 */
    uint32_t rmc_last_update_ms; /**< 最近一次 RMC 更新时间，ms。 */
    uint16_t rmc_period_ms;      /**< 最近 RMC Sample 周期，ms。 */
    uint16_t rmc_age_ms;         /**< 当前 RMC Sample Age，ms，超范围时饱和。 */

    uint8_t rmc_velocity_valid;          /**< RMC Speed/Course Sample 本身是否有效。 */
    uint8_t gps_position_velocity_valid; /**< Position-window Velocity 是否有效。 */
    uint8_t gps_velocity_valid;          /**< 当前选定 Velocity Observation 是否有效。 */
    uint8_t gps_velocity_source;         /**< 当前 Velocity Source，NavGpsVelocitySource_t。 */

    uint8_t gps_velocity_control_ready; /**< Velocity Observation 是否满足闭环控制质量要求。 */
    uint8_t gps_position_control_ready; /**< GPS Position 是否满足 Position Hold 质量要求。 */
} NavState_t;

/**
 * @brief 双 IMU 冗余健康状态。
 */
typedef struct
{
    uint8_t active_imu_sel;         /**< 当前 Active IMU：0=IMU1，1=IMU2。 */
    uint8_t imu1_healthy;           /**< IMU1 健康状态：0=异常，1=健康。 */
    uint8_t imu2_healthy;           /**< IMU2 健康状态：0=异常，1=健康。 */

    uint16_t bad_frame_count;       /**< 当前连续异常观测计数。 */
    uint16_t good_frame_count;      /**< 当前连续健康观测计数。 */

    uint8_t dual_fault; /**< 双路均不可可靠使用时置 1。 */
} ImuHealthStatus_t;

/**
 * @brief 关键 Task Heartbeat 时间戳。
 *
 * IWDG Feed Task 根据各字段距离当前 Tick 的 Age
 * 判断对应任务是否仍在正常运行。
 */
typedef struct
{
    volatile uint32_t flightctrl_last_tick; /**< FlightCtrl Task 最近一次 Heartbeat，RTOS Tick。 */
    volatile uint32_t powermonit_last_tick; /**< PowerMonit Task 最近一次 Heartbeat，RTOS Tick。 */

    /* 后续关键 Task 接入 IWDG Watchdog 时在此扩展。 */
} SystemHeartbeat_t;

/**
 * @brief 电源监测与 Current Limiter 共享状态。
 */
typedef struct
{
    bool voltage_fault; /**< Critical Voltage Fault 是否已确认。 */

    bool current_limiting;           /**< Current Limiter 当前是否正在介入。 */
    uint16_t current_limit_permille; /**< Collective Throttle Scale，600~1000 对应 60%~100%。 */
    float current_filtered_a;        /**< 滤波后的整机总电流，A。 */
} PowerHealth_t;

/**
 * @brief 飞行器 Arm 状态。
 */
typedef enum
{
    ARM_STATE_DISARMED = 0, /**< 电机输出处于安全锁定状态。 */
    ARM_STATE_ARMED = 1,    /**< 已解锁，允许正常飞行控制输出。 */
} ArmState_t;

/**
 * @brief Indicator Task 消费的系统提示事件。
 */
typedef enum
{
    EVT_ARMED = 0,    /**< 进入 Armed。 */
    EVT_DISARMED = 1, /**< 进入 Disarmed。 */

    EVT_LOW_BATTERY = 2,      /**< Low Battery Early Warning。 */
    EVT_CRITICAL_BATTERY = 3, /**< Critical Battery Fault。 */

    EVT_GPS_FIX_ACQUIRED = 4, /**< GPS Home 首次成功建立。 */
    EVT_SD_CARD_ERROR = 5,    /**< SD / Blackbox 数据链异常。 */
    EVT_IMU_FAULT = 6,        /**< 双 IMU 冗余故障。 */
    EVT_RC_LOST = 7,          /**< 已建立过连接后发生 RC Lost。 */
    EVT_CURRENT_LIMITING = 8, /**< Current Limiter 正在介入。 */

    EVT_RC_CALIB_STARTED = 9,  /**< RC Calibration 开始。 */
    EVT_RC_CALIB_SUCCESS = 10, /**< RC Calibration 成功。 */
    EVT_RC_CALIB_FAILED = 11,  /**< RC Calibration 失败。 */

    EVT_GYRO_CALIB_SUCCESS = 12, /**< 启动 Gyro Bias Calibration 完成。 */

    EVT_LEVEL_TRIM_STARTED = 13, /**< Level Trim 开始采样。 */
    EVT_LEVEL_TRIM_SUCCESS = 14, /**< Level Trim 保存并生效成功。 */
    EVT_LEVEL_TRIM_FAILED = 15,  /**< Level Trim 失败，旧配置保持不变。 */

    EVT_MAG_CAL_CAPTURE_STARTED = 16, /**< Mag Calibration 数据采集开始。 */
    EVT_MAG_CAL_CAPTURE_DONE = 17,    /**< Mag Calibration 数据采集正常结束。 */
    EVT_MAG_CAL_CAPTURE_FAILED = 18,  /**< Mag Calibration 数据采集失败。 */
} IndicatorEvent_t;

/**
 * @brief Nav Task 控制命令。
 */
typedef enum
{
    NAV_CMD_SET_HOME = 0, /**< 请求重新建立 GPS Home。 */
} NavCommand_t;

/*
 * 跨模块共享运行状态。
 *
 * 这些对象由对应功能模块写入、其他 Task 读取；
 * volatile 用于确保每次访问实际发生在内存，不代表完整的 RTOS 同步机制。
 */
extern volatile ImuHealthStatus_t g_imu_health;
extern volatile SystemHeartbeat_t g_heartbeat;
extern volatile ArmState_t g_arm_state;
extern volatile PowerHealth_t g_power_health;

#endif
