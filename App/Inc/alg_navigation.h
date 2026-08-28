/**
 * @file    alg_navigation.h
 * @brief   水平状态估计、速度控制及位置控制接口。
 */

#ifndef __ALG_NAVIGATION_H
#define __ALG_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * 是否启用 IMU 水平加速度对 Estimator Velocity 的预测。
 * 
 * 当前阶段仅影响状态估计与诊断数据，Velocity Hold 是否接管控制，
 * 仍由 app_flightCtrl.c 独立决定。
 */
#define HORIZONTAL_ESTIMATOR_IMU_PREDICTION_ENABLED 1U

/** RMC Course 可用于构造 N/E 速度向量的最低 Ground Speed，m/s。*/
#define NAV_RMC_VECTOR_MIN_SPEED_MPS 0.25f

/**
 * @brief   水平 N/E 状态估计器。
 * 
 * 同一维护水平 Velocity、Position、Acceleration、Accel Bias
 * 以及 GPS 融合相关状态和诊断信息。
 */
typedef struct
{
    float velocity_n_mps;   /**< 北向估计速度，m/s。 */
    float velocity_e_mps;   /**< 东向估计速度，m/s。 */
    float position_n_m;     /**< 北向估计位置，m。 */
    float position_e_m;     /**< 东向估计位置，m。 */

    float gps_position_n_m; //*< 最近 GPS Position 的局部北向坐标，m。 *
    float gps_position_e_m; //*< 最近 GPS Position 的局部东向坐标，m。 *

    float accel_n_mps2;     /**< 去除 Bias 后的北向水平加速度，m/s^2。 */
    float accel_e_mps2;     /**< 去除 Bias 后的东向水平加速度，m/s^2。 */
    float accel_lpf_n_mps2; /**< 低通滤波后的北向原始加速度，m/s^2。 */
    float accel_lpf_e_mps2; /**< 低通滤波后的东向原始加速度，m/s^2。 */

    /*
     * Accel Bias 的持久状态保存在机体系。
     * 这样可避免 Yaw 改变后，将固定机体偏差误解释为新的 N/E Bias。
     */
    float accel_bias_forward_mps2; /**< 机头向前 Accel Bias，m/s^2。 */
    float accel_bias_right_mps2;   /**< 机头向右 Accel Bias，m/s^2。 */

    /*
     * N/E Bias 是机体系 Bias 在当前 Yaw 下的投影，
     * 不作为独立持久状态，仅用于 Estimator 输出和 Blackbox 诊断。
     */
    float accel_bias_n_mps2; /**< 北向 Accel Bias 投影，m/s^2。 */
    float accel_bias_e_mps2; /**< 东向 Accel Bias 投影，m/s^2。 */

    /* GPS Position局部坐标参考系。 */
    int32_t position_reference_lat_e7;  /**< 参考纬度，1e-7 deg。 */
    int32_t position_reference_lon_e7;  /**< 参考纬度，1e-7 deg。 */
    float meter_per_lat_e7;             /**< 纬度每 1e-7 deg 对应距离，m。 */
    float meter_per_lon_e7;             /**< 经度每 1e-7 deg 对应距离，m。 */

    /* Position alpha-beta修正状态机诊断。 */
    float position_sample_elapsed_s;            /**< 距上一帧 GPS Position 样本的累计时间，s。 */
    float position_velocity_correction_n_mps;   /**< 本次 Position Beta 修正实际施加的北向加速度修正量，m/s。 */
    float position_velocity_correction_e_mps;   /**< 本次 Position Beta 修正实际施加的东向加速度修正量，m/s。 */

    uint32_t last_gps_sequence; /**< 最后处理的 GPS Velocity 样本序号。 */
    uint32_t last_gps_tick_ms;  /**< 最后接收的 GPS Velocity 样本时间戳，ms。 */
    uint32_t gps_accept_count;  /**< GPS Velocity 成功融合次数。 */
    uint32_t gps_reject_count;  /**< GPS Velocity 拒绝次数。 */
    uint8_t gps_accept_streak;          /**< 当前连续接受 GPS Velocity 样本的次数。 */

    uint32_t last_position_sequence;    /**< 最后处理的 GPS Position 样本序号。 */
    uint32_t position_accept_count;     /**< GPS Position 样本累计接受次数。 */
    uint32_t position_reject_count;     /**< GPS Position 样本累计拒绝次数。 */
    uint32_t position_velocity_correction_count;    /**< Position beta 速度修正累计执行次数。 */
    uint8_t position_accept_streak;     /**< 当前连续接受 GPS Position 样本的次数。 */

    uint8_t initialized;       /**< Velocity Estimator 是否完成初始对齐。 */
    uint8_t gps_healthy;       /**< GPS Velocity 更新是否健康且未超时。 */
    uint8_t last_gps_accepted; /**< 最近到达的 GPS Velocity 样本是否被接受。 */
    uint8_t last_rmc_vector_used;

    uint8_t position_initialized;       /**< 是否已建立有效的局部 Position 参考系。 */
    uint8_t last_position_accepted;     /**< 最近 GPS Position 样本是否接受。 */
    uint8_t last_position_rejected;     /**< 最近 GPS Position 样本是否因有效性或 Innovation 条件被拒绝。 */
    uint8_t last_position_velocity_correction_applied;      /**< 最近一次 Position 修正是否同时修正了 Velocity。 */
} HorizontalEstimator_t;

/**
 * @brief   水平速度控制器状态。
 * 
 * 使用 N/E Velocity PID 生成水平 Acceleration 请求，
 * 再根据当前 Yaw 转换为机体系 Roll/Pitch 目标角。
 */
typedef struct
{
    float kp;                 /**< Velocity 比例增益。 */
    float ki;                 /**< Velocity 积分增益。 */
    float kd;                 /**< 实测水平 Acceleration 阻尼增益。 */
    float arw_gain;           /**< Anti-windup 回算增益。 */
    uint8_t integral_enabled; /**< 本周期是否允许 Integral 学习。 */

    float accel_requested_n_mps2;   /**< 限幅前北向 Acceleration 请求，m/s^2。 */
    float accel_requested_e_mps2;   /**< 限幅前东向 Acceleration 请求，m/2^2。 */

    float integral_accel_n_mps2; /**< Integral 对北向 Acceleration 的贡献，m/s^2。 */
    float integral_accel_e_mps2; /**< Integarl 对东向 Acceleration 的贡献，m/s^2。 */

    float accel_target_n_mps2; /**< 最终北向目标 Acceleration，m/s^2。 */
    float accel_target_e_mps2; /**< 最终东向目标 Acceleration，m/s^2。 */

    float p_accel_n_mps2;   /**< 北向比例项产生的 Acceleration 分量，m/s^2。 */
    float p_accel_e_mps2;   /**< 东向比例项产生的 Acceleration 分量，m/s^2。 */
    float d_accel_n_mps2;   /**< 北向阻尼项产生的 Acceleration 分量，m/s^2。 */
    float d_accel_e_mps2;   /**< 东向阻尼项产生的 Acceleration 分量，m/s^2。 */

    float roll_target_deg;  /**< Roll 目标角，deg。 */
    float pitch_target_deg; /**< Pitch 目标角，deg。 */

    uint8_t output_limited; /**< Acceleration 或 Tilt 输出是否饱和。 */
} VelocityController_t;

/**
 * @brief   Velocity Integral 更新模式。
 */
typedef enum
{
    VELOCITY_INTEGRAL_MODE_FROZEN = 0,      /**< Integral 完全冻结。 */
    VELOCITY_INTEGRAL_MODE_UNLOAD_ONLY = 1, /**< 仅允许已有 Integral 卸载。 */
    VELOCITY_INTEGRAL_MODE_LEARN = 2,       /**< 正常学习 Integral。 */
} VelocityIntegralMode_t;

/**
 * @brief   Position Control 状态机阶段。
 */
typedef enum
{
    POSITION_CONTROL_PHASE_INACTIVE = 0,    /**< 未启用。 */
    POSITION_CONTROL_PHASE_MOVING = 1,      /**< Pilot 指令驱动移动。 */
    POSITION_CONTROL_PHASE_BRAKING = 2,     /**< 松杆后的制动阶段。 */
    POSITION_CONTROL_PHASE_HOLD = 3,        /**< Position Hold。 */
} PositionControlPhase_t;

/**
 * @brief   水平位置控制器状态。
 */
typedef struct
{
    float kp; /**< Position 外环比例增益。 */

    float target_n_m; /**< 北向目标位置，m。 */
    float target_e_m; /**< 东向目标位置，m。 */
    float error_n_m;  /**< 北向位置误差，m。 */
    float error_e_m;  /**< 东向位置误差，m。 */

    float velocity_target_n_mps;    /**< 北向目标速度，m/s。 */
    float velocity_target_e_mps;    /**< 东向目标速度，m/s。 */

    float brake_elapsed_time_s;             /**< 当前 BRAKING 阶段已持续时间，s。 */
    uint32_t brake_last_gps_sequence;       /**< BRAKING 阶段最后处理的 GPS 样本序号。 */
    uint8_t brake_low_speed_sample_count;   /**< BRAKING 阶段连续满足低速条件的 GPS 样本数。 */

    uint32_t integral_learning_last_gps_sequence;           /**< Integral 学习判定最后处理的 GPS 样本序号。 */
    uint32_t integral_learning_candidate_start_tick_ms;     /**< Integral 学习候选状态开始时间，ms。 */
    uint8_t integral_learning_valid_sample_count;           /**< 连续满足 Integral 学习条件的有效 GPS 样本数。 */
    uint8_t integral_learning_allowed;                      /**< 当前是否允许 velocity Integral 正常学习。 */

    PositionControlPhase_t phase;       /**< 当前 Position Control 状态机阶段。 */
    uint8_t initialized;                /**< 是否已完成 Position Controller 初始化。 */
} PositionController_t;

extern HorizontalEstimator_t horizontal_estimator;
extern VelocityController_t velocity_controller;
extern PositionController_t position_controller;

/**
 * @brief 初始化水平状态估计器。
 */
void HorizontalEstimator_Init(void);

/**
 * @brief 将水平状态估计器复位至初始状态。
 */
void HorizontalEstimator_Reset(void);

/**
 * @brief 使用 IMU 与当前姿态预测 N/E 水平状态。
 *
 * 将机体系 Accel 转换至导航系并用于水平 Acceleration/Velocity 预测。
 *
 * @param[in] ax_g      IMU X 轴加速度，g。
 * @param[in] ay_g      IMU Y 轴加速度，g。
 * @param[in] az_g      IMU Z 轴加速度，g。
 * @param[in] roll_rad  当前 Roll，rad。
 * @param[in] pitch_rad 当前 Pitch，rad。
 * @param[in] yaw_rad   当前导航 Yaw，rad。
 * @param[in] dt        预测周期，s。
 * @param[in] learn_accel_bias  是否允许学习 Accel Bias。
 * 
 * @pre learn_accel_bias 仅应在 Disarmed，GPS有效且机体近似静止时启用。
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
 * @brief   使用新的 RMC Velocity 样本修正水平速度状态。
 *
 * 有效低速 RMC 仍用于刷新 GPS Health，并使用 Ground Speed 大小约束；
 * Estimator Velocity 模长：由于低速 Course 不可靠，此时不使用其 N/E 方向。
 * 
 *
 * @param[in]   gps_velocity_n_mps              GPS 北向速度，m/s。
 * @param[in]   gps_velocity_e_mps              GPS 东向速度，m/s。
 * @param[in]   gps_sequence                    GPS 样本序号。
 * @param[in]   gps_tick_ms                     GPS 样本时间戳，ms。
 * @param[in]   yaw_rad                         当前导航 Yaw，rad。
 * @param[in]   sample_valid                    GPS Velocity 样本是否有效。
 * @param[in]   rmc_vector_use_allowed          是否允许使用 RMC N/E 速度方向。
 * @param[in]   allow_accel_bias_correction     是否允许根据 GPS Innovation 修正 Accel Bias。
 * @param[in]   allow_state_reacquire           是否允许长时间失联后重新对齐 Estimator 状态。
 *
 * @return  true 表示 样本被接收并刷新 GPS health；
 *          false 表示 样本非法、过期或因 innovation 超限被拒绝。
 */
bool HorizontalEstimator_CorrectGps(float gps_velocity_n_mps,
                                    float gps_velocity_e_mps,
                                    uint32_t gps_sequence,
                                    uint32_t gps_tick_ms,
                                    float yaw_rad,
                                    bool sample_valid,
                                    bool rmc_vector_use_allowed,
                                    bool allow_accel_bias_correction,
                                    bool allow_state_reacquire);

/**
 * @brief   清除 Estimator 的局部 Position 参考系。
 * 
 * @note    不清除 Velocity 和 Accel bias，用于退出 Position Hold
 *          或重新捕获锚点。
 */
void HorizontalEstimator_ResetPosition(void);

/**
 * @brief   以当前 GPS 坐标建立局部 N/E Position 参考系。
 * 
 * 建立参考系后当前位置定义为局部坐标原点。
 * 
 * @return  true 表示建立成功；false 表示样本序号或坐标非法。
 */
bool HorizontalEstimator_SetPositionReference(int32_t latitude_e7,
                                              int32_t longitude_e7,
                                              uint32_t gps_sequence);

/**
 * @brief   使用 GPS Position 对 Estimator Position/Velocity 进行 alpha-beta 修正。
 * 
 * @param[in] latitude_e7       GPS 纬度，1e-7 deg。
 * @param[in] longitude_e7      GPS 经度，1e-7 deg。
 * @param[in] gps_sequence      GPS Position 样本序号。
 * @param[in] sample_valid      样本质量与时效是否满足要求。
 * @param[in] allow_state_reacquire 是否允许重新捕获 Position 状态。
 * 
 * @return  true 表示样本已接收并完成修正；false 表示未就绪或样本被拒绝。
 */
bool HorizontalEstimator_CorrectPosition(int32_t latitude_e7,
                                         int32_t longitude_e7,
                                         uint32_t gps_sequence,
                                         bool sample_valid,
                                         bool allow_state_reacquire);

/**
 * @brief 根据 GPS 样本时效更新健康状态。
 * 
 * @param[in] now_ms  当前系统时间，ms。
 */
void HorizontalEstimator_UpdateHealth(uint32_t now_ms);

/**
 * @brief   判断 GPS 融合状态是否健康可用。
 * 
 * @param[in] now_ms  当前系统时间，ms。
 * @return  true 表示 Estimator 已初始化且 GPS 更新未超时。
 */
bool HorizontalEstimator_IsHealthy(uint32_t now_ms);

/**
 * @brief   初始化水平速度控制器。
 */
void VelocityController_Init(void);

/**
 * @brief   复位水平速度控制器。
 */
void VelocityController_Reset(void);

/**
 * @brief   仅清除 Velocity Controller 的 Integral。
 *
 * @note    保留当前 Acceleration 输出及 Roll/Pitch Slew 状态，
 *          便面人工接管时姿态目标突变。
 */
void VelocityController_ResetIntegral(void);

/**
 * @brief   运行 N/E Velocity PID 并生成 Roll/Pitch 目标角。
 *
 * @param[in]   velocity_target_n_mps   北向目标速度，m/s。
 * @param[in]   velocity_target_e_mps   东向目标速度，m/s。
 * @param[in]   velocity_meas_n_mps     北向实测速度，m/s。
 * @param[in]   velocity_meas_e_mps     东向实测速度，m/s。
 * @param[in]   accel_meas_n_mps2       北向实测水平加速度，m/s^2。
 * @param[in]   accel_meas_e_mps2       东向实测水平加速度，m/s^2。
 * @param[in]   yaw_rad                 当前导航 Yaw，rad。
 * @param[in]   integral_mode           Integral 更新模式。
 * @param[in]   dt                      控制周期，s。
 */
void VelocityController_Update(float velocity_target_n_mps,
                               float velocity_target_e_mps,
                               float velocity_meas_n_mps,
                               float velocity_meas_e_mps,
                               float accel_meas_n_mps2,
                               float accel_meas_e_mps2,
                               float yaw_rad,
                               VelocityIntegralMode_t integral_mode,
                               float dt);

/**
 * @brief   初始化 Position Controller。
 */
void PositionController_Init(void);

/**
 * @brief   重置 Position Controller。
 */
void PositionController_Reset(void);

/**
 * @brief   使用当前 Estimator Position 进入 Position Control。
 * 
 * @param[in] horizontal_state  当前水平状态估计。
 * @return  true 表示初始化成功；false 表示 Position 状态无效。
 */
bool PositionController_Enter(const HorizontalEstimator_t *horizontal_state);

/**
 * @brief   更新 Position Control 状态机并生成 Velocity Target。
 *
 * BRAKING 阶段仅在连续收到有效且相互一致的新 GPS 观测后才捕获 Hold 锚点;
 * 不使用超时强制捕获仍在移动中的位置
 *
 * @param[in,out]   horizontal_state        当前水平状态估计。
 * @param[in]       pilot_velocity_n_mps    Pilot 北向 Velocity Feedforward，m/s。
 * @param[in]       pilot_velocity_e_mps    Pilot 东向 Velocity Feedforward，m/s。
 * @param[in]       brake_velocity_observation_consistent   RMC Velocity 与 Position-window Velocity 是否有效且一致。
 * @param[in]       brake_observed_speed_mps                两者速度观测中较大的 Velocity 模长，m/s。
 * @param[in]       dt  控制周期，s。
 *
 * @retval  true 表示本周期更新成功；
 *          false 表示状态未就绪，dt非法或 Position 越界。
 */
bool PositionController_Update(HorizontalEstimator_t *horizontal_state,
                               float pilot_velocity_n_mps,
                               float pilot_velocity_e_mps,
                               bool brake_velocity_observations_consistent,
                               float brake_observed_speed_mps,
                               float dt);

#endif
