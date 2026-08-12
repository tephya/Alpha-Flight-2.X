#ifndef __BSP_BLACKBOX_H
#define __BSP_BLACKBOX_H

#include <stdint.h>

/*====== 帧同步：每条记录的第一字节固定是这个magic，供上位机解析器丢失同步后重新扫描定位 ======
 * 记录格式统一为：[MAGIC(1B)][type(1B)][...记录内容...]
 * 后续解析对不上号时，逐字节向后扫描找下一个MAGIC字节，从哪里重新按type分发，
 * 代价是每条记录多1字节开销*/
#define BB_FRAME_MAGIC 0xAAU

/*====== 帧类型tag，写入文件的每条记录第一字节，供离线解析脚本区分记录种类 ======*/
typedef enum
{
    BB_REC_MOTION = 0x01,        // 高频运动帧
    BB_REC_ARM_CHANGED = 0x02,       // 低频事件：解锁状态变化
    BB_REC_DUAL_FAULT = 0x03,    // 低频事件：双IMU失效
    BB_REC_VOLTAGE_FAULT = 0x04, // 低频事件：持续低压
    BB_REC_IMU_SWITCH = 0x05,    // 低频事件：主备IMU切换
    BB_REC_CONTROL = 0x06,      // PID调参控制帧：旧71-byte payload
    BB_REC_CONTROL_V2 = 0x07,   // CONTROL V2：增加Yaw角目标和Yaw模式
    BB_REC_CONTROL_V3 = 0x08,   // 融合Yaw和Level Trim可观测诊断帧
    BB_REC_MAG_CAL_SAMPLE = 0x09,   // MAG校准数据采样事件
    BB_REC_CONTROL_V4 = 0x0A,       // V4：增加MAG拒绝原因和参考磁场
    BB_REC_NAVIGATION = 0x0B,       // GPS/IMU水平速度Estimator与Velocity Controller诊断
    BB_REC_NAVIGATION_V2 = 0x0C,    // 增加GPS位置与Position Controller诊断
    BB_REC_NAVIGATION_V3 = 0x0D,
    BB_REC_NAVIGATION_V4 = 0x0E,
    BB_REC_NAVIGATION_V5 = 0x0F,    // 增加Position Mode阶段状态
} BB_RecType_t;

typedef __packed struct
{
    uint32_t timestamp_cycle; // DWT 时间戳，解析时按uint32_t无符号差值处理

    int16_t angle_cdeg[3];        // Roll/Pitch/Yaw，0.01°
    int16_t angle_target_cdeg[3]; // Roll/Pitch/Yaw目标，0.01°

    int16_t level_trim_offset_cdeg[2];      // active IMU Roll/Pitch Trim，0.01°
    int16_t mag_yaw_cdeg;        // 最近一次倾斜补偿后的原始Mag航向，0.01°
    int16_t yaw_mag_innovation_cdeg;        // wrap(Mag_Yaw - fused_Yaw)，0.01°
    uint16_t mag_field_mG;       // Mag三轴模长，milli-gauss
    uint16_t mag_field_reference_mG;    // 当前参考磁场
    uint16_t mag_field_ratio_permille;  // norm/reference
    uint8_t mag_reject_reason;          // YawMagRejectReason_t位掩码

    int16_t rate_target_ddps[3]; // 目标角速度，0.1°/s
    int16_t rate_meas_ddps[3];   // Gyro角速度，0.1°/s

    int16_t p_term_duint[3]; // Rate PID P项，0.1 Mixer unit
    int16_t i_term_duint[3]; // Rate PID I项
    int16_t d_term_duint[3]; // Rate PID D项
    int16_t output_duint[3]; // Rate PID最终输出s

    uint16_t motor[4];  // Mixer输出，尚未加DSHOT 48
    int16_t current_dA; // 滤波总电流，0.1A
    uint16_t current_limit_permille;

    uint16_t throttle;
    uint16_t limited_throttle;
    uint16_t control_dt_us;
    uint8_t active_imu;
    uint8_t fresh_imu_flags;

    uint8_t flags; // bit0=Airmode，bit1=Current limiting，bit2=Yaw manual mode
                    // bit3=Level Trim ready，bit4=Mag accepted，bit5=Yaw estimator initialized
} BB_ControlData_t;

typedef __packed struct
{
    uint32_t timestamp_cycle;
    uint32_t rmc_sequence;

    int16_t gps_velocity_n_cms;     // cm/s
    int16_t gps_velocity_e_cms;
    int16_t est_velocity_n_cms;
    int16_t est_velocity_e_cms;

    int16_t accel_n_cms2;           // cm/s^2
    int16_t accel_e_cms2;
    int16_t accel_bias_n_cms2;
    int16_t accel_bias_e_cms2;

    int16_t velocity_target_n_cms;
    int16_t velocity_target_e_cms;
    int16_t nav_roll_target_cdeg;
    int16_t nav_pitch_target_cdeg;
    int16_t yaw_cdeg;
    int16_t controller_i_n_cms2;
    int16_t controller_i_e_cms2;

    uint16_t gps_age_ms;
    uint16_t rmc_period_ms;
    uint16_t gps_hdop_centi;
    uint8_t gps_satellites;

    /* bit0=GPS velocity valid, bit1=Estimator initialized,
     * bit2=Estimator healthy, bit3=Velocity Hold requested,
     * bit4=Velocity Hold active, bit5=最近GPS校正接收
     * bit6=IMU prediction compiled-in, bit7=GPS control ready */
    uint8_t flags;

    int16_t position_n_cm;
    int16_t position_e_cm;
    int16_t gps_position_n_cm;
    int16_t gps_position_e_cm;
    int16_t position_target_n_cm;
    int16_t position_target_e_cm;
    int16_t position_error_n_cm;
    int16_t position_error_e_cm;

    /* bit0: GPS Position control ready
     * bit1: Position Hold requested
     * bit2: Position Hold active
     * bit3: Position Controller initialized
     * bit4: Position control output enabled
     * bit5: latest GPS Position correction accepted
     * bit6: latest GPS Position correction rejected
     * bit7: latest accepted GPS Position innovation also corrected Velocity */
    uint8_t position_flags;

    /* 0=Manual，1=Velocity Hold，2=Position Hold */
    uint8_t horizontal_mode;

    int16_t rmc_velocity_n_cms;
    int16_t rmc_velocity_e_cms;
    int16_t gps_position_velocity_n_cms;
    int16_t gps_position_velocity_e_cms;
    uint8_t gps_velocity_source;
    uint8_t position_control_phase;
} BB_NavigationData_t;


int8_t BB_Init(void);
void BB_BufferInit(void);
void BB_ControlInit(void);

/*====== 生产者接口：各Task/模块调用，写入一条记录 ======*/
/* 高频运动帧：由Task_FlightCtrl每控制周期调用 */
int8_t BB_LogMotion(uint16_t time_ms, const int16_t angle_cdeg[3],
                    const uint16_t motor[4], const int8_t target_cdeg[3]);
int8_t BB_LogControl(const BB_ControlData_t *data);
int8_t BB_LogNavigation(const BB_NavigationData_t *data);

/* 低频事件帧，只在状态跳变时调用 */
int8_t BB_LogArmChanged(uint16_t time_ms, uint8_t armed);
int8_t BB_LogDualFault(uint16_t time_ms);
int8_t BB_LogVoltageFault(uint16_t time_ms);
int8_t BB_LogImuSwitch(uint16_t time_ms, uint8_t new_active_imu);
int8_t BB_LogMagCalibration(uint32_t timestamp_cycle,
                            float mag_x_gauss,
                            float mag_y_gauss,
                            float mag_z_gauss);

int8_t BB_WaitReady(uint32_t timeout_ms);
int8_t BB_Process(void);
int8_t BB_Close(void);
uint8_t BB_GetErrorFlags(void);

/*====== 跨Task控制请求：由Task_FlightCtrl/app_arm.c这类实时任务调用，
 * 只是非阻塞地置一个事件位，真正执行关闭/开新文件的动作在Task_Blackbox里做，
 * 不能让ARM状态机直接调用BB_Close/BB_Init这类阻塞式SD卡操作 ====== */
#define BB_CTRL_CLOSE_REQ (1U << 0) // 请求关闭当前文件，用于ARM->Disarm(含紧急disarm)时
#define BB_CTRL_NEWFILE_REQ (1U << 1) // 请求开一个新日志文件，用于Disarm->ARM时

void BB_RequestClose(void);
void BB_RequestNewFile(void);
uint32_t BB_PollControlRequest(uint32_t timeout_ms);

#endif
