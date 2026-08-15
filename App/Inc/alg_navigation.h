#ifndef __ALG_NAVIGATION_H
#define __ALG_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

/* 置1后Estimator使用IMU水平加速度在两次GPS样本之间推进速度，
 * 第一阶段只影响诊断数据：Velocity Hold控制接管仍由app_flightctrl.c单独开关 */
#define HORIZONTAL_ESTIMATOR_IMU_PREDICTION_ENABLED 1U

#define NAV_RMC_VECTOR_MIN_SPEED_MPS 0.25f

typedef struct
{
    /* 统一水平状态：Velocity与Position均由Estimator持有。 */
    float velocity_n_mps; // 北向估计速度 (m/s)
    float velocity_e_mps; // 东向估计速度
    float position_n_m;
    float position_e_m;

    /* 最近一帧原始GPS Position在局部坐标系中的位置。 */
    float gps_position_n_m;
    float gps_position_e_m;

    float accel_n_mps2;     // 消除零偏后的北向加速度 (m/s^2)
    float accel_e_mps2;     // 消除零偏后的东向加速度
    float accel_lpf_n_mps2; // 低通滤波后的北向原始加速度 (m/s^2)
    float accel_lpf_e_mps2; // 低通滤波后的东向原始加速度

    /*
     * Accel bias的持久状态保存在机体系，避免飞行器改变Yaw后，
     * 同一个机体固定参数被错误解释成另一组N/E偏差。
     */
    float accel_bias_forward_mps2; // 机头向前方向的水平加速度偏差
    float accel_bias_right_mps2;   // 机头向右方向的水平加速度偏差

    /*
     * 下面两项不是独立状态，而是上述机体系bias在当前Yaw下的N/E投影。
     * 保留他们用于Estimator输出和现有Blackbox日志。
     */
    float accel_bias_n_mps2; // 北向加速度计零偏估计值 (m/s^2)
    float accel_bias_e_mps2; // 东向加速度计零偏估计值

    /* GPS Position局部坐标参考系。 */
    int32_t position_reference_lat_e7;
    int32_t position_reference_lon_e7;
    float meter_per_lat_e7;
    float meter_per_lon_e7;

    /* Position alpha-beta修正状态机诊断。 */
    float position_sample_elapsed_s;
    float position_velocity_correction_n_mps;
    float position_velocity_correction_e_mps;

    uint32_t last_gps_sequence; // 最后一个处理的GPS样本序列号，防重复数据
    uint32_t last_gps_tick_ms;  // 最后一个被接收的GPS样本时间戳 (ms)
    uint32_t gps_accept_count;  // 成功融合的GPS样本总计数
    uint32_t gps_reject_count;  // 因新息过大或非法而拒绝的GPS样本计数

    uint8_t gps_accept_streak;

    uint32_t last_position_sequence;
    uint32_t position_accept_count;
    uint32_t position_reject_count;
    uint32_t position_velocity_correction_count;

    uint8_t position_accept_streak;

    uint8_t initialized;       // 估计器是否已完成首帧GPS速度的初始对齐
    uint8_t gps_healthy;       // 当前GPS速度更新是否正常且未超时
    uint8_t last_gps_accepted; // 上一帧到达的GPS样本是否被滤波器接受
    uint8_t last_rmc_vector_used;

    uint8_t position_initialized;
    uint8_t last_position_accepted;
    uint8_t last_position_rejected;
    uint8_t last_position_velocity_correction_applied;
} HorizontalEstimator_t;

typedef struct
{
    float kp;                 // 速度环比例增益
    float ki;                 // 速度环积分增益
    float kd;                 // 实测水平Acceleration阻尼增益
    float arw_gain;           // 饱和Acceleration error折算为Velocity error的增益
    uint8_t integral_enabled; // 1=本周期允许由Velocity error学习Integral

    /* 未经限幅的PID请求，用于诊断和Anti-windup */
    float accel_requested_n_mps2;
    float accel_requested_e_mps2;

    /* Integral直接保存其对水平加速度指令的贡献，单位m/s^2 */
    float integral_accel_n_mps2; // 北向加速度积分项累积量
    float integral_accel_e_mps2; // 东向加速度积分项累积量

    float accel_target_n_mps2; // 输出的北向目标加速度 (m/s^2)
    float accel_target_e_mps2; // 输出的东向目标加速度

    float p_accel_n_mps2;
    float p_accel_e_mps2;
    float d_accel_n_mps2;
    float d_accel_e_mps2;

    float roll_target_deg;  // 转换出的目标横滚角指令 (deg)
    float pitch_target_deg; // 转换出的目标俯仰角指令

    uint8_t output_limited; // 1=Acceleration/tilt权限已饱和
} VelocityController_t;

typedef enum
{
    POSITION_CONTROL_PHASE_INACTIVE = 0,
    POSITION_CONTROL_PHASE_MOVING = 1,
    POSITION_CONTROL_PHASE_BRAKING = 2,
    POSITION_CONTROL_PHASE_HOLD = 3,
} PositionControlPhase_t;

typedef struct
{
    float kp; // 位置外环比例增益

    float target_n_m; // 期望控制前往的北向目标位置 (m)
    float target_e_m; // 期望控制前往的东向目标位置
    float error_n_m;  // 当前北向位置偏差 (m)
    float error_e_m;  // 当前东向位置偏差

    float velocity_target_n_mps; // 前馈与位置修正叠加输出的北向目标速度 (m/s)
    float velocity_target_e_mps; // 前馈与位置修正叠加输出的东向目标速度

    float brake_elapsed_time_s; // 本轮BRAKING已经持续的总时间
    uint32_t brake_last_gps_sequence;
    uint8_t brake_low_speed_sample_count;

    uint32_t integral_learning_last_gps_sequence;
    uint32_t integral_learning_candidate_start_tick_ms;
    uint8_t integral_learning_valid_sample_count;
    uint8_t integral_learning_allowed;

    PositionControlPhase_t phase;
    uint8_t initialized; // 位置控制器是否已建立有效的局部参考系
} PositionController_t;

extern HorizontalEstimator_t horizontal_estimator;
extern VelocityController_t velocity_controller;
extern PositionController_t position_controller;

/**
 * @brief   初始化水平速度与加速度估计器状态
 */
void HorizontalEstimator_Init(void);

/**
 * @brief   重置水平状态估计器所有内部变量至零态
 */
void HorizontalEstimator_Reset(void);

/**
 * @brief   使用本轮IMU与姿态推进N/E水平速度。
 * @param   learn_accel_bias 仅允许在Disarmed、GPS有效且近似静止时置true。
 */
void HorizontalEstimator_Predict(float ax_g,
                                 float ay_g,
                                 float az_g,
                                 float roll_rad,
                                 float pitch_rad,
                                 float yaw_rad,
                                 float dt,
                                 bool learn_accel_bias);

/**
 * @brief   消费一条新的RMC ground-speed/course样本。
 *
 * @note    有效的低速RMC仍刷新GPS health,并使用ground speed大小约束
 *          Estimator Velocity模长;由于course不可信,不使用N/E方向.
 *          低速方向信息由GPS Position beta修正提供.
 *
 * @param   gps_velocity_n_mps  GPS北向速度
 * @param   gps_velocity_e_mps  GPS东向速度
 * @param   gps_sequence        GPS样本序号
 * @param   gps_tick_ms         GPS样本时间戳
 * @param   yaw_rad             当前导航Yaw；用于N/E与机体系bias之间的旋转
 * @param   sample_valid        当前GPS速度样本是否有效
 * @param   allow_accel_bias_correction 是否允许利用GPS innovation
 *                                      慢速修正飞行中的Accel bias
 *
 * @return  true=样本接收；false=样本质量不合格或innovation过大。
 */
bool HorizontalEstimator_CorrectGps(float gps_velocity_n_mps,
                                    float gps_velocity_e_mps,
                                    uint32_t gps_sequence,
                                    uint32_t gps_tick_ms,
                                    float yaw_rad,
                                    bool sample_valid,
                                    bool allow_accel_bias_correction,
                                    bool allow_state_reacquire);

/**
 * @brief   清除Estimator的局部Position参考系及Position修正诊断。
 * @note    不清除Velocity和Accel bias；用于退出Position Hold或重新捕获锚点。
 */
void HorizontalEstimator_ResetPosition(void);

/**
 * @brief   以当前GPS坐标建立局部N/E参考系，初始Position为零。
 * @return  true=参考系建立成功；false=sequence或坐标非法。
 */
bool HorizontalEstimator_SetPositionReference(int32_t latitude_e7,
                                              int32_t longitude_e7,
                                              uint32_t gps_sequence);

/**
 * @brief   使用新GPS Position对Estimator的Position与Velocity做alpha-beta校正。
 * @param   sample_valid    GPS Position质量与时效是否满足控制要求。
 * @return  true=Position样本已接收并完成修正；false=未就绪或样本被拒绝。
 */
bool HorizontalEstimator_CorrectPosition(int32_t latitude_e7,
                                         int32_t longitude_e7,
                                         uint32_t gps_sequence,
                                         bool sample_valid,
                                         bool allow_state_reacquire);

/**
 * @brief   基于系统运行时间戳检查GPS样本是否超时，以此更新GPS健康状态
 * @param   now_ms  当前系统时间(ms)
 */
void HorizontalEstimator_UpdateHealth(uint32_t now_ms);

/**
 * @brief   判断当前状态估计器的GPS融合是否健康可用
 * @param   now_ms  当前系统时间(ms)
 * @return  true=已初始化且接收正常；false=未初始化或接收超时
 */
bool HorizontalEstimator_IsHealthy(uint32_t now_ms);

/**
 * @brief   初始化PI控制器各项增益与变量
 */
void VelocityController_Init(void);

/**
 * @brief   重置速度控制器内部的积分项与最终指令输出
 */
void VelocityController_Reset(void);

/**
 * @brief   仅清除Velocity Controller的Integral
 *
 * @note    保留当前目标加速度和Roll/Pitch Slew状态，避免人工接管时姿态目标突跳到零。
 */
void VelocityController_ResetIntegral(void);

/**
 * @brief   平滑衰减 Velocity Controller 已有积分。
 * 
 * @note    用于 Position Hold 的 MOVING/BRAKING 阶段。它只改变Integral，
 *          不重置当前 Acceleration target；后续输出仍受原有 Slew limiter 约束。
 * 
 * @param   dt  控制周期(s)
 */
void VelocityController_DecayIntegral(float dt);

/**
 * @brief   运行N/E 速度 PID，输出体系Roll/Pitch目标角。
 *
 * @param   velocity_target_n_mps   N向Velocity target
 * @param   velocity_target_e_mps   E向Velocity target
 * @param   velocity_meas_n_mps     N向估计Velocity
 * @param   velocity_meas_e_mps     E向估计Velocity
 * @param   accel_meas_n_mps2       N向实测水平Acceleration
 * @param   accel_meas_e_mps2       E向实测水平Acceleration
 * @param   yaw_rad                 导航Yaw
 * @param   allow_integral_learning true=允许Velocity error建立抗风Integral;
 *                                  false=禁止建立新Integral，仅允许反向误差和ARW卸载已有Integral
 * @param   dt                      控制周期
 */
void VelocityController_Update(float velocity_target_n_mps,
                               float velocity_target_e_mps,
                               float velocity_meas_n_mps,
                               float velocity_meas_e_mps,
                               float accel_meas_n_mps2,
                               float accel_meas_e_mps2,
                               float yaw_rad,
                               bool allow_integral_learning,
                               float dt);

/**
 * @brief   初始化位置比例控制器各项增益与变量
 */
void PositionController_Init(void);

/**
 * @brief   重置位置控制器内部的参考坐标原点与偏差状态
 */
void PositionController_Reset(void);

/**
 * @brief   使用Estimator当前Position进入Position模式状态机。
 * @return  true=坐标有效且初始化成功；false=坐标越界。
 */
bool PositionController_Enter(const HorizontalEstimator_t *horizontal_state);

/**
 * @brief   运行Position模式三阶段状态机并生成Velocity target。
 *
 * @note    BRAKING只在收到连续,有效且相互一致的新GPS观测后才捕获HOLD锚点;
 *          不再通过超时强制捕获仍在移动中的位置
 *
 * @param   horizontal_state    水平状态估计器
 * @param   pilot_velocity_n_mps    驾驶员北向Velocity前馈
 * @param   pilot_velocity_e_mps    驾驶员东向Velocity前馈
 * @param   brake_velocity_observation_consistent
 *          true=本轮RMC Velocity与Position-window Velocity均有效且差值可接受
 * @param   brake_observed_speed_mps    RMC与Position-window两者中较大的Velocity模长
 * @param   dt  控制周期
 *
 * @retval  false=状态未就绪、dt异常或位置越界。
 */
bool PositionController_Update(HorizontalEstimator_t *horizontal_state,
                               float pilot_velocity_n_mps,
                               float pilot_velocity_e_mps,
                               bool brake_velocity_observations_consistent,
                               float brake_observed_speed_mps,
                               float dt);

#endif
