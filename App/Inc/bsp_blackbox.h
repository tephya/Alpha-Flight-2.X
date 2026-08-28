/**
 * @file    bsp_blackbox.h
 * @brief   Blackbox 二进制日志记录，缓冲及文件控制接口。
 */

#ifndef __BSP_BLACKBOX_H
#define __BSP_BLACKBOX_H

#include <stdint.h>

/*
 * Blackbox Record 同步字节。
 *
 * 每条记录统一采用：
 *
 * [MAGIC][TYPE][PAYLOAD...]
 *
 * 离线解析器失去同步后可以逐字节搜索 MAGIC，
 * 再根据 TYPE 重新识别后续记录。
 */
#define BB_FRAME_MAGIC 0xAAU

/*
 * Blackbox 跨 Task 控制请求。
 *
 * FlightCtrl / Arm 等实时路径只能发送非阻塞 Request，
 * 真正的 File Open / Flush / Close 均由 Blackbox Task 执行。
 */
#define BB_CTRL_CLOSE_REQ (1U << 0)   /**< 请求关闭当前日志文件。 */
#define BB_CTRL_NEWFILE_REQ (1U << 1) /**< 请求创建新的日志文件。 */

/**
 * @brief Blackbox Record 类型。
 */
typedef enum
{
    BB_REC_MOTION = 0x01,        /**< 高频 Motion Record。 */
    BB_REC_ARM_CHANGED = 0x02,   /**< Arm 状态变化事件。 */
    BB_REC_DUAL_FAULT = 0x03,    /**< Dual IMU Fault 事件。 */
    BB_REC_VOLTAGE_FAULT = 0x04, /**< Critical Voltage Fault 事件。 */
    BB_REC_IMU_SWITCH = 0x05,    /**< Active IMU 切换事件。 */

    BB_REC_CONTROL = 0x06,    /**< 旧版 Control Record。 */
    BB_REC_CONTROL_V2 = 0x07, /**< Control V2。 */
    BB_REC_CONTROL_V3 = 0x08, /**< Control V3。 */

    BB_REC_MAG_CAL_SAMPLE = 0x09, /**< Mag Calibration 原始样本。 */

    BB_REC_CONTROL_V4 = 0x0A, /**< 当前 Control V4 Record。 */

    BB_REC_NAVIGATION = 0x0B,    /**< 旧版 Navigation Record。 */
    BB_REC_NAVIGATION_V2 = 0x0C, /**< Navigation V2。 */
    BB_REC_NAVIGATION_V3 = 0x0D, /**< Navigation V3。 */
    BB_REC_NAVIGATION_V4 = 0x0E, /**< Navigation V4。 */
    BB_REC_NAVIGATION_V5 = 0x0F, /**< 当前 Navigation V5 Record。 */
} BB_RecType_t;

/**
 * @brief Control V4 日志 Payload。
 *
 * @note 使用 Packed Layout 直接写入二进制文件；
 *       修改字段顺序、类型或数量时必须同步更新离线解析器。
 */
typedef __packed struct
{
    uint32_t timestamp_cycle; /**< DWT Cycle 时间戳。 */

    int16_t angle_cdeg[3];        /**< Roll/Pitch/Yaw，0.01 deg。 */
    int16_t angle_target_cdeg[3]; /**< Roll/Pitch/Yaw Target，0.01 deg。 */

    int16_t level_trim_offset_cdeg[2]; /**< Active IMU Roll/Pitch Level Trim，0.01 deg。 */

    int16_t mag_yaw_cdeg;            /**< 最近一次倾斜补偿后的 Mag Yaw，0.01 deg。 */
    int16_t yaw_mag_innovation_cdeg; /**< wrap(Mag Yaw - Fused Yaw)，0.01 deg。 */

    uint16_t mag_field_mG;             /**< 当前 Mag 三轴模长，mG。 */
    uint16_t mag_field_reference_mG;   /**< 当前参考磁场模长，mG。 */
    uint16_t mag_field_ratio_permille; /**< 当前 Mag Field / Reference，千分比。 */
    uint8_t mag_reject_reason;         /**< YawMagRejectReason_t 位掩码。 */

    int16_t rate_target_ddps[3]; /**< Roll/Pitch/Yaw Rate Target，0.1 deg/s。 */
    int16_t rate_meas_ddps[3];   /**< Gyro Roll/Pitch/Yaw Rate，0.1 deg/s。 */

    int16_t p_term_duint[3]; /**< Rate PID P Term，0.1 Mixer Unit。 */
    int16_t i_term_duint[3]; /**< Rate PID I Term，0.1 Mixer Unit。 */
    int16_t d_term_duint[3]; /**< Rate PID D Term，0.1 Mixer Unit。 */
    int16_t output_duint[3]; /**< Rate PID 最终输出，0.1 Mixer Unit。 */

    uint16_t motor[4];  /**< Mixer 输出，尚未叠加 DShot Offset。 */

    int16_t current_dA; /**< 滤波后的整机电流，0.1 A。 */
    uint16_t current_limit_permille; /**< Collective Throttle Scale，千分比。 */

    uint16_t throttle;         /**< Current Limiter 前的基础 Throttle。 */
    uint16_t limited_throttle; /**< Current Limiter 后的基础 Throttle。 */

    uint16_t control_dt_us; /**< 当前 Control Dt，us。 */

    uint8_t active_imu;      /**< 当前 Active IMU。 */
    uint8_t fresh_imu_flags; /**< 本轮 Fresh IMU 位标志。 */

    /**
     * Control 状态位：
     * bit0 = Airmode Active
     * bit1 = Current Limiting
     * bit2 = Yaw Manual Rate Mode
     * bit3 = Level Trim Ready
     * bit4 = 当前 Mag Sample Accepted
     * bit5 = Yaw Estimator Initialized
     */
    uint8_t flags;
} BB_ControlData_t;

/**
 * @brief Navigation V5 日志 Payload。
 *
 * @note 使用 Packed Layout 直接写入二进制文件；
 *       修改字段时必须同步更新离线解析器。
 */
typedef __packed struct
{
    uint32_t timestamp_cycle; /**< DWT Cycle 时间戳。 */
    uint32_t rmc_sequence;    /**< 当前 RMC Sequence。 */

    int16_t gps_velocity_n_cms; /**< 当前选定 GPS North Velocity，cm/s。 */
    int16_t gps_velocity_e_cms; /**< 当前选定 GPS East Velocity，cm/s。 */

    int16_t est_velocity_n_cms; /**< Horizontal Estimator North Velocity，cm/s。 */
    int16_t est_velocity_e_cms; /**< Horizontal Estimator East Velocity，cm/s。 */

    int16_t accel_n_cms2; /**< 水平 North Acceleration，cm/s²。 */
    int16_t accel_e_cms2; /**< 水平 East Acceleration，cm/s²。 */

    int16_t accel_bias_n_cms2; /**< Estimator North Accel Bias，cm/s²。 */
    int16_t accel_bias_e_cms2; /**< Estimator East Accel Bias，cm/s²。 */

    int16_t velocity_target_n_cms; /**< North Velocity Target，cm/s。 */
    int16_t velocity_target_e_cms; /**< East Velocity Target，cm/s。 */

    int16_t nav_roll_target_cdeg;  /**< Navigation Roll Target，0.01 deg。 */
    int16_t nav_pitch_target_cdeg; /**< Navigation Pitch Target，0.01 deg。 */
    int16_t yaw_cdeg;              /**< 当前 Navigation Yaw，0.01 deg。 */

    int16_t controller_i_n_cms2; /**< Velocity Controller North I Term，cm/s²。 */
    int16_t controller_i_e_cms2; /**< Velocity Controller East I Term，cm/s²。 */

    uint16_t gps_age_ms;     /**< 当前 GPS Velocity Sample Age，ms。 */
    uint16_t rmc_period_ms;  /**< RMC Sample Period，ms。 */
    uint16_t gps_hdop_centi; /**< GPS HDOP × 100。 */
    uint8_t gps_satellites;  /**< 当前可用 Satellite 数量。 */

    /**
     * Velocity / Estimator 状态位：
     * bit0 = GPS Velocity Valid
     * bit1 = Estimator Initialized
     * bit2 = Estimator Healthy
     * bit3 = Velocity Hold Requested
     * bit4 = Velocity Hold Active
     * bit5 = 最近一次 GPS Velocity Correction Accepted
     * bit6 = IMU Prediction Compiled In
     * bit7 = GPS Velocity Control Ready
     */
    uint8_t flags;

    int16_t position_n_cm; /**< Estimator North Position，cm。 */
    int16_t position_e_cm; /**< Estimator East Position，cm。 */

    int16_t gps_position_n_cm; /**< 当前 GPS North Local Position，cm。 */
    int16_t gps_position_e_cm; /**< 当前 GPS East Local Position，cm。 */

    int16_t position_target_n_cm; /**< Position Controller North Target，cm。 */
    int16_t position_target_e_cm; /**< Position Controller East Target，cm。 */

    int16_t position_error_n_cm; /**< North Position Error，cm。 */
    int16_t position_error_e_cm; /**< East Position Error，cm。 */

    /**
     * Position Controller 状态位：
     * bit0 = GPS Position Control Ready
     * bit1 = Position Hold Requested
     * bit2 = Position Hold Active
     * bit3 = Position Controller Initialized
     * bit4 = Position Control Output Enabled
     * bit5 = 最近一次 GPS Position Correction Accepted
     * bit6 = 最近一次 GPS Position Correction Rejected
     * bit7 = 最近一次 Accepted Position Innovation 同时修正了 Velocity
     */
    uint8_t position_flags;

    uint8_t horizontal_mode; /**< 0=Manual，1=Velocity Hold，2=Position Hold。 */

    int16_t rmc_velocity_n_cms; /**< RMC North Velocity，cm/s。 */
    int16_t rmc_velocity_e_cms; /**< RMC East Velocity，cm/s。 */

    int16_t gps_position_velocity_n_cms; /**< Position-window North Velocity，cm/s。 */
    int16_t gps_position_velocity_e_cms; /**< Position-window East Velocity，cm/s。 */

    uint8_t gps_velocity_source;    /**< NavGpsVelocitySource_t。 */
    uint8_t position_control_phase; /**< Position Controller 当前阶段。 */
} BB_NavigationData_t;

/**
 * @brief 挂载文件系统并创建新的 Blackbox 日志文件。
 * 
 * @return 0  创建成功。
 * @return -1 FatFS Mount 失败。
 * @return -2 文件创建失败。
 * 
 * @note 包含 FatFS 文件系统操作，只能由 Blackbox Task 调用。
 */
int8_t BB_Init(void);

/**
 * @brief 初始化双缓冲生产者/消费者状态。
 */
void BB_BufferInit(void);

/**
 * @brief 初始化 Blackbox 跨 Task 控制事件对象。
 */
void BB_ControlInit(void);

/*
 * Blackbox Record 生产接口。
 *
 * 这些函数只把完整 Record 复制到 RAM Buffer，
 * 不直接执行 SD Card 文件写入。
 */

/**
 * @brief 写入一条 Motion Record。
 */
int8_t BB_LogMotion(uint16_t time_ms, const int16_t angle_cdeg[3],
                    const uint16_t motor[4], const int8_t target_cdeg[3]);

/**
 * @brief 写入一条 Control V4 Record。
 */
int8_t BB_LogControl(const BB_ControlData_t *data);

/**
 * @brief 写入一条 Navigation V5 Record。
 */
int8_t BB_LogNavigation(const BB_NavigationData_t *data);

/**
 * @brief 写入 Arm 状态变化事件。
 */
int8_t BB_LogArmChanged(uint16_t time_ms, uint8_t armed);

/**
 * @brief 写入 Dual IMU Fault 事件。
 */
int8_t BB_LogDualFault(uint16_t time_ms);

/**
 * @brief 写入 Critical Voltage Fault 事件。
 */
int8_t BB_LogVoltageFault(uint16_t time_ms);

/**
 * @brief 写入 Active IMU 切换事件。
 */
int8_t BB_LogImuSwitch(uint16_t time_ms, uint8_t new_active_imu);

/**
 * @brief 写入一条未经 Hard/Soft-Iron 校正的 Mag Calibration Sample。
 */
int8_t BB_LogMagCalibration(uint32_t timestamp_cycle,
                            float mag_x_gauss,
                            float mag_y_gauss,
                            float mag_z_gauss);

/**
 * @brief 阻塞等待至少一块 Buffer 进入 Ready 状态。
 *
 * @param[in] timeout_ms  最大等待时间，ms。
 *
 * @return 0  有 Buffer 待写。
 * @return -1 超时或 Semaphore Wait 失败。
 */
int8_t BB_WaitReady(uint32_t timeout_ms);

/**
 * @brief 消费一块 Ready Buffer 并写入 SD Card。
 *
 * @return 1  本次处理了一块 Buffer。
 * @return 0  当前没有 Ready Buffer。
 */
int8_t BB_Process(void);

/**
 * @brief 写出当前未满 Buffer 的尾部数据并关闭文件。
 *
 * @return 0  成功。
 * @return -1 尾部数据写入失败。
 * @return -2 文件关闭失败。
 */
int8_t BB_Close(void);

/**
 * @brief 查询本次日志运行期间的错误状态。
 *
 * @return bit0=1 表示发生过 Buffer Overflow；
 *         bit1=1 表示发生过 SD Write Error。
 */
uint8_t BB_GetErrorFlags(void);

/**
 * @brief 非阻塞请求关闭当前日志文件。
 */
void BB_RequestClose(void);

/**
 * @brief 非阻塞请求创建新的日志文件。
 */
void BB_RequestNewFile(void);

/**
 * @brief 等待或轮询 Blackbox 控制请求。
 *
 * @param[in] timeout_ms  0 表示非阻塞检查，否则为最大等待时间。
 *
 * @return 命中的 BB_CTRL_* 位；
 *         CMSIS-RTOS2 EventFlags Error Code 表示本次无有效请求。
 */
uint32_t BB_PollControlRequest(uint32_t timeout_ms);

#endif
