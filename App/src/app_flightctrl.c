/**
 * @file    app_flightctrl.c
 * @brief   飞行控制主循环，水平辅助模式与控制链路实现。
 * 
 * Active IMU 新帧构成成本模块唯一的控制周期基准。
 * 每个有效周期依次完成姿态估计，Yaw 融合，水平状态估计，
 * ARM 状态更新，串级控制，Mixer，电机输出及日志记录。
 */

#include "app_flightctrl.h"
#include "app_imu2_redundancy.h"
#include "app_imu_calibration.h"
#include "app_rc_link.h"
#include "app_arm.h"
#include "app_level_trim.h"
#include "app_shared_types.h"
#include "app_mag_calibration.h"
#include "bsp_elrs.h"
#include "bsp_dshot.h"
#include "bsp_qmc5883.h"
#include "bsp_icm42688.h"
#include "bsp_blackbox.h"
#include "alg_attitude.h"
#include "alg_controller.h"
#include "alg_pid.h"
#include "alg_mixer.h"
#include "alg_yaw_estimator.h"
#include "alg_navigation.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <math.h>

/*
 * 当前阶段 Airmode 使用迟滞门限，
 * 防止油门在单一阈值附近波动导致姿态控制反复启停。
 */
#define AIRMODE_ACTIVATION_THROTTLE 30U     // Airmode 激活油门阈值。
#define AIRMODE_DEACTIVATION_THROTTLE 20U   // Airmode 退出油门阈值。

#define FLIGHT_ACCEL_BIAS_SETTLE_TIME_S 1.0f    // 飞行中 Accel Bias 修正前的稳定确认时间，s。

#define VELOCITY_HOLD_CONTROL_ENABLE 1U     // Velocity Hold 控制输出总开关。   
#define POSITION_HOLD_CONTROL_ENABLE 1U     // Position Hold 控制输出总开关。

#define HORIZONTAL_MODE_RC_CHANNEL 8U       // 水平模式开关通道，CRSF CH9。
#define HORIZONTAL_MODE_SC_MIDDLE_MIN 700U  // SC 中档有效区间下限。
#define HORIZONTAL_MODE_SC_MIDDLE_MAX 1300U // SC 中当有效区间上限。
#define HORIZONTAL_MODE_SC_HIGH_MIN 1600U   // SC 高档判定下限。

#define HORIZONTAL_PILOT_ACCEL_LIMIT_MPS2 6.0f  // Pilot Velocity Command 最大二维变化率，m/s^2。

#define HORIZONTAL_MODE_ENTRY_MAX_VELOCITY_ERROR_MPS 0.50f  // 辅助模式切入允许的最大 Velocity Estimator Error，m/s。
#define HORIZONTAL_MODE_ENTRY_MAX_POSITION_ERROR_M 1.00f    // Position Hold 切入允许的最大 Positionn Error，m。
#define HORIZONTAL_MODE_ENTRY_GPS_ACCEPT_STREAK 3U          // 切入辅助模式要求的连续有效 GPS Velocity 样本数。
#define HORIZONTAL_MODE_ENTRY_POSITION_ACCEPT_STREAK 3U     // Position Hold 切入要求的连续有效 Position 样本数。
#define HORIZONTAL_MODE_BLEND_TIME_S 0.50f                  // 水平控制目标从 Manual 平滑接管的 Blend 时间，s。

#define POSITION_BRAKE_GPS_VELOCITY_DISAGREEMENT_MAX_MPS 0.30f  // RMC 与 Position-window Velocity 最大允许差异，m/s。

#define HORIZONTAL_BIAS_STATIONARY_TIME_S 1.0f      // Disarmed Accel Bias 学习前要求的静止时间，s。
#define HORIZONTAL_BIAS_GYRO_MAX_DPS 3.0f           // 静止判断允许的最大各轴 Gyro 绝对值，deg/s。
#define HORIZONTAL_BIAS_ACCEL_NORM_MIN_G 0.85f      // 静止判断允许的最小 Accel 模长，g。
#define HORIZONTAL_BIAS_ACCEL_NORM_MAX_G 1.15f      // 静止判断允许的最大 Accel 模长，g。
#define HORIZONTAL_BIAS_GPS_SPEED_MAX_MPS 0.15f     // 静止 Bias 学习允许的最大 GPS Speed，m/s。

#define VELOCITY_HOLD_MAX_SPEED_MPS 2.0f            // Velocity Hold 最大 Pilot Velocity，m/s。
#define POSITION_HOLD_PILOT_SPEED_MPS 1.50f         // Position Hold 最大 Pilot Velocity，m/s。
#define HORIZONTAL_MODE_ENTRY_MIN_THROTTLE 400U     // 水平辅助模式允许接管的最低油门。

#define HORIZONTAL_PILOT_INPUT_MAX_DEG 30.0f        // 水平摇杆映射后的最大输入角度，deg。
#define HORIZONTAL_PILOT_DEADZONE_DEG 2.0f          // 水平摇杆中心 Deadzone，deg。
#define HORIZONTAL_PILOT_EXPO 0.50f                 // 水平摇杆 Expo 系数。

#define RMC_VECTOR_CONFIRM_REQUIRED_SAMPLES 2U          // Position Hold 使用 RMC Vector 前要求的连续确认样本数。
#define RMC_VECTOR_CONFIRM_POSITION_SPEED_MIN_MPS 0.10f // 用于确认 RMC 方向的最小 Position-window Speed，m/s。
#define RMC_VECTOR_CONFIRM_MAX_DISAGREEMENT_MPS 0.30f   // 两种 GPS Velocity 观测的最大允许差异，m/s。

/*
 * RMC Course 以 True North 为基准，而当前 Mag Yaw 以 Magnetic North 为基准
 * 东磁偏角为正：True Heading = Magnetic Heading + Declination
 */
#define NAV_MAG_DECLINATION_DEG 0.0f    // 当前飞行地点磁偏角，deg。

#define BB_NAV_LOG_DIVIDER 10U          // Navigation 日志相对于 Control 日志的分频系数。
#define NAV_SHADOW_FORCE_DSHOT_ZERO 0U  // 置 1 时保留完整控制计算但强制 DSHOT 输出为零。

#include "usart.h"
#include <stdio.h>

extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t RCChannelMailboxHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t NavStateMailboxHandle;

/**
 * @brief   水平飞行控制模式。
 */
typedef enum
{
    HORIZONTAL_MODE_MANUAL = 0,         /**< Manual：Roll/Pitch 直接来自 RC。 */
    HORIZONTAL_MODE_VELOCITY_HOLD = 1,  /**< Velocity Hold：摇杆控制水平速度。 */
    HORIZONTAL_MODE_POSITION_HOLD = 2,  /**< Position Hold：松杆后保持局部位置。 */
} FlightHorizontalMode_t;

static PID_t pid_yaw;       // Heading Hold Yaw 外环。

static FlightControl_t fc = {0};

static RCChannelData_t s_rc_last = {0}; // 最近一次有效 RC 数据：Mailbox 使用非阻塞读取，因此需保留本地状态。

static NavState_t s_nav_last = {0};     // Navigation Task 最近一次发布的数据快照。

/* requested 表示 RC 请求，active 表示控制器实际已经接管。 */
static uint8_t s_velocity_hold_requested;
static uint8_t s_velocity_hold_active;
static uint8_t s_position_hold_requested;
static uint8_t s_position_hold_active;

static FlightHorizontalMode_t s_horizontal_mode_active = HORIZONTAL_MODE_MANUAL;
static FlightHorizontalMode_t s_horizontal_mode_last_request = HORIZONTAL_MODE_MANUAL;

/*
 * 记录是否已经真实观察到 SC Manual 档。
 * 辅助模式异常退出后必须先回 Manual，禁止条件恢复后自动重新接管。
 */
static uint8_t s_horizontal_manual_seen;

/* 当前水平控制器使用的 N/E Velocity Target，m/s。 */
static float s_velocity_target_n_mps;
static float s_velocity_target_e_mps;

/* 经 Deadzone，Expo 和 Slew Rate 整形后的 Pilot Velocity Command，m/s。 */
static float s_pilot_velocity_cmd_n_mps;
static float s_pilot_velocity_cmd_e_mps;

/* Position Hold 稳定后允许飞行中 Accel Bias 修正的确认计时，s。 */
static float s_flight_accel_bias_hold_time_s;

/* Disarmed 静止状态持续时间，用于允许 Accel Bias 学习，s。 */
static float s_disarmed_accel_bias_stationary_time_s;

/* Manual -> Horizontal Control 接管时的目标混合比例，[0,1]。 */
static float s_horizontal_control_blend;

/* Position Hold 中对 RMC Velocity Vector 的连续一致性确认状态。 */
static uint8_t s_rmc_vector_confirm_count;
static uint8_t s_rmc_vector_confirmed;

/* 清除 RMC Velocity Vector 的连续一致性确认状态。 */
static void FlightControl_ResetRmcVectorConfirmation(void)
{
    s_rmc_vector_confirm_count = 0U;
    s_rmc_vector_confirmed = 0U;
}

/*
 * 初始化完整飞行控制算法链。
 * Mag Calibration 必须先于 Yaw Estimator 初始化，
 * 以便将降准得到的参考磁场模长作为上电 Field Reference。
 */
void FlightControl_Init(void)
{
    AngleController_Init();
    RateController_Init();
    HorizontalEstimator_Init();
    VelocityController_Init();
    PositionController_Init();

    /* 先初始化Mag Cali，再把参考场强传给YawEstimator */
    MagCalibration_Init();
    YawEstimator_Init(MagCalibration_GetFieldReferenceGauss());

    /* 最大输出限制为 30°/s，避免航向误差直接要求过大的 Yaw Rate */
    PID_Init(&pid_yaw, 1.0f, 0.0f, 0.0f, 30.0f);

    fc.yaw_mode = 0;
    fc.yaw_target = 0.0f;
    fc.airmode_active = 0U;

    s_disarmed_accel_bias_stationary_time_s = 0.0f;
    s_horizontal_control_blend = 0.0f;
    
    FlightControl_ResetRmcVectorConfirmation();
}

/*
 * 清除控制器动态状态并返回 Manual 水平模式。
 * 
 * Horizontal Estimator 不在这里复位，
 * 使 Disarmed 状态下仍能持续观察 Navigation Estimation。
 */
static void FlightControl_Reset(void)
{
    AngleController_Reset();
    RateController_Reset();

    fc.roll_target = 0.0f;
    fc.pitch_target = 0.0f;
    fc.yaw_rate_target = 0.0f;
    fc.roll_rate_target = 0.0f;
    fc.pitch_rate_target = 0.0f;
    fc.roll_cmd = 0.0f;
    fc.pitch_cmd = 0.0f;
    fc.yaw_cmd = 0.0f;

    fc.yaw_target = fc.yaw_meas;
    fc.yaw_mode = 0;

    PID_Reset(&pid_yaw);

    /* 
     * 只复位Velocity Controller，不复位Estimator；
     * 否则Disarmed步行测试无法观察速度估计。 
     */
    VelocityController_Reset();
    PositionController_Reset();

    s_horizontal_mode_active = HORIZONTAL_MODE_MANUAL;
    s_horizontal_mode_last_request = HORIZONTAL_MODE_MANUAL;
    s_horizontal_manual_seen = 0U;

    s_velocity_hold_requested = 0U;
    s_velocity_hold_active = 0U;
    s_position_hold_requested = 0U;
    s_position_hold_active = 0U;

    s_velocity_target_n_mps = 0.0f;
    s_velocity_target_e_mps = 0.0f;

    s_flight_accel_bias_hold_time_s = 0.0f;

    s_pilot_velocity_cmd_n_mps = 0.0f;
    s_pilot_velocity_cmd_e_mps = 0.0f;

    s_horizontal_control_blend = 0.0f;
    FlightControl_ResetRmcVectorConfirmation();
}

/*
 * 仅重建 PID Derivative 的历史采样基准。
 * 
 * 用于控制周期异常后的恢复；保留 Integral 与上一周期 Output，
 * 下一次更更新通过 first_update 对齐 Measurement，
 * 避免跨越异常 Dt 产生 Derivative Spike。
 */
static void PID_ResetDerivativeOnly(PID_t *pid)
{
    pid->derivative = 0.0f;
    pid->derivative_raw = 0.0f;
    pid->derivative_lpf = 0.0f;
    pid->first_update = 1U;
}

/*
 * 判断当前 RMC Velocity Vector 是否允许参与水平状态修正。
 * 
 * Manual / Velocity Hold 只要求 RMC 本身满足速度有效性；
 * Position Hold 额外要求 RMC Vector 与 Position-window Velocity
 * 连续保持方向和模长一致，降低低速 Course 抖动导致误修正的风险。
 */
static bool FlightControl_UpdateRmcVectorUseAllowed(bool require_position_confirmation)
{
    if(s_nav_last.gps_velocity_valid == 0U)
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }

    const float rmc_speed_mps = sqrtf(
        s_nav_last.rmc_velocity_n_mps *
            s_nav_last.rmc_velocity_n_mps +
        s_nav_last.rmc_velocity_e_mps *
            s_nav_last.rmc_velocity_e_mps);

    if(!(rmc_speed_mps >= 0.0f && rmc_speed_mps <= 30.0f))
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }

    /*
     * 低速时 Ground Speed 仍有一定参考价值，
     * 但 RMC Course 方向噪声较大，因此禁止使用 N/E Velocity Vector。
     */
    if (rmc_speed_mps < NAV_RMC_VECTOR_MIN_SPEED_MPS)
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }

    // Manual 与 Velocity Hold 不要求 Position-window Velocity 二次确认。
    if (!require_position_confirmation)
    {
        FlightControl_ResetRmcVectorConfirmation();
        return true;
    }

    if (s_nav_last.gps_position_velocity_valid == 0U)
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }

    const float position_speed_mps = sqrtf(
        s_nav_last.gps_position_velocity_n_mps *
            s_nav_last.gps_position_velocity_n_mps +
        s_nav_last.gps_position_velocity_e_mps *
            s_nav_last.gps_position_velocity_e_mps);

    /*
     * Position-window Velocity 过小时自身方向也缺乏可信度，
     * 此时不能用于确认 RMC Course。
     */
    if (!(position_speed_mps >=
          RMC_VECTOR_CONFIRM_POSITION_SPEED_MIN_MPS))
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }

    const float disagreement_n_mps =
        s_nav_last.rmc_velocity_n_mps -
        s_nav_last.gps_position_velocity_n_mps;

    const float disagreement_e_mps =
        s_nav_last.rmc_velocity_e_mps -
        s_nav_last.gps_position_velocity_e_mps;

    const float disagreement_mps = sqrtf(
        disagreement_n_mps * disagreement_n_mps +
        disagreement_e_mps * disagreement_e_mps);

    /*
     * Dot Product > 0 表示两组速度观测总体同向，
     * 再结合二维速度差门限判断两者是否一致。
     */
    const float direction_dot =
        s_nav_last.rmc_velocity_n_mps *
            s_nav_last.gps_position_velocity_n_mps +
        s_nav_last.rmc_velocity_e_mps *
            s_nav_last.gps_position_velocity_e_mps;

    const bool candidates_consistent =
        disagreement_mps <=
            RMC_VECTOR_CONFIRM_MAX_DISAGREEMENT_MPS &&
        direction_dot > 0.0f;

    if (!candidates_consistent)
    {
        FlightControl_ResetRmcVectorConfirmation();
        return false;
    }
    
    /*
     * 要求连续多个样本保持一致后才确认 RMC Vector，
     * 避免单帧偶然吻合直接获得 Position Hold 修正权限。
     */
    if(s_rmc_vector_confirm_count <
        RMC_VECTOR_CONFIRM_REQUIRED_SAMPLES)
    {
        s_rmc_vector_confirm_count++;
    }

    if (s_rmc_vector_confirm_count >=
        RMC_VECTOR_CONFIRM_REQUIRED_SAMPLES)
    {
        s_rmc_vector_confirmed = 1U;
    }

    return s_rmc_vector_confirmed != 0U;
}

/*
 * 更新水平 Navigation Estimator。
 * 
 * 每个 Active IMU 周期执行 IMU Prediction；
 * GPS Velocity / Position 仅在检测到新序号时参与修正。
 * 
 * 同时负责 Disarmed Accel Bias 学习，飞行中 Bias 慢修正
 * 以及 GPS Position 失效后的局部参考系管理。
 */
static void FlightControl_UpdateHorizontalEstimator(const IcmData_t *imu, float dt)
{
    NavState_t nav;

    if (osMessageQueueGet(NavStateMailboxHandle, &nav, NULL, 0U) == osOK)
    {
        s_nav_last = nav;
    }

    /*
     * 水平导航坐标系依赖有效 Yaw。
     * 未建立绝对航向前不能可靠完成 Body -> N/E 坐标变换。
     */
    if (!YawEstimator_IsInitialized())
    {
        HorizontalEstimator_Reset();
        FlightControl_ResetRmcVectorConfirmation();
        return;
    }

/* ----------------------------- Disarmed Accel Bias 学习 ---------------------------- */

    const float gps_speed = sqrtf(
        s_nav_last.gps_velocity_n_mps * s_nav_last.gps_velocity_n_mps +
        s_nav_last.gps_velocity_e_mps * s_nav_last.gps_velocity_e_mps);

    const float accel_norm_sq =
        imu->ax * imu->ax +
        imu->ay * imu->ay +
        imu->az * imu->az;

    const float accel_norm_min_sq =
        HORIZONTAL_BIAS_ACCEL_NORM_MIN_G *
        HORIZONTAL_BIAS_ACCEL_NORM_MIN_G;

    const float accel_norm_max_sq =
        HORIZONTAL_BIAS_ACCEL_NORM_MAX_G *
        HORIZONTAL_BIAS_ACCEL_NORM_MAX_G;

    /*
     * 同时使用 Gyro 与 Accel 模长判断 IMU 是否处于静止状态。
     * 不直接要求三个 Accel 分量固定，以为机体可能以任意静止姿态放置。
     */
    const bool imu_stationary =
        fabsf(imu->gx) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        fabsf(imu->gy) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        fabsf(imu->gz) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        accel_norm_sq >= accel_norm_min_sq &&
        accel_norm_sq <= accel_norm_max_sq;

    /*
     * Disarmed Accel Bias 学习必须同时满足：
     * GPS 表明机体基本静止，且 IMU 自身也符合静止特征。
     * 避免将真实运动 Acceleration 学入 Sensor Bias。
     */
    const bool disarmed_bias_candidate =
        (g_arm_state == ARM_STATE_DISARMED) &&
        (s_nav_last.gps_velocity_valid != 0U) &&
        (gps_speed < HORIZONTAL_BIAS_GPS_SPEED_MAX_MPS) &&
        imu_stationary;

    if (disarmed_bias_candidate && dt >= 0.0005f && dt <= 0.0050f)
    {
        if (s_disarmed_accel_bias_stationary_time_s <
            HORIZONTAL_BIAS_STATIONARY_TIME_S)
        {
            s_disarmed_accel_bias_stationary_time_s += dt;
            if (s_disarmed_accel_bias_stationary_time_s >
                HORIZONTAL_BIAS_STATIONARY_TIME_S)
            {
                s_disarmed_accel_bias_stationary_time_s =
                    HORIZONTAL_BIAS_STATIONARY_TIME_S;
            }
        }
    }
    else
    {
        s_disarmed_accel_bias_stationary_time_s = 0.0f;
    }

    const bool learn_accel_bias =
        s_disarmed_accel_bias_stationary_time_s >=
        HORIZONTAL_BIAS_STATIONARY_TIME_S;

    /*
     * Navigation 使用 True North 坐标系，
     * 因此需要在 Magnetic Yaw 上叠加当地磁偏角。
     */
    const float navigation_yaw_rad =
        attitude.yaw + NAV_MAG_DECLINATION_DEG * 0.0174532925f;

    HorizontalEstimator_Predict(imu->ax,
                                imu->ay,
                                imu->az,
                                attitude.roll,
                                attitude.pitch,
                                navigation_yaw_rad,
                                dt,
                                learn_accel_bias);

/* ----------------------------- 飞行中 Bias 慢修正 ---------------------------- */

    /*
     * 比较两套独立 GPS Velocity Observation；
     * RMC Ground Speed/Course 与 Position-window Velocity。
     */
    const float gps_velocity_candidate_disagreement_n_mps =
        s_nav_last.rmc_velocity_n_mps -
        s_nav_last.gps_position_velocity_n_mps;

    const float gps_velocity_candidate_disagreement_e_mps =
        s_nav_last.rmc_velocity_e_mps -
        s_nav_last.gps_position_velocity_e_mps;

    const float gps_velocity_candidate_disagreement_mps = sqrtf(
        gps_velocity_candidate_disagreement_n_mps *
            gps_velocity_candidate_disagreement_n_mps +
        gps_velocity_candidate_disagreement_e_mps *
            gps_velocity_candidate_disagreement_e_mps);

    /*
     * 飞行中的 Accel Bias 修正只允许在 Position Hold 已稳定进入 HOLD，
     * 且两套 GPS Velocity Observation 保持一致时启用。
     * 
     * 这样可降低真实机动过程或 GPS 瞬态被学习为 IMU Bias 的风险。
     */
    const bool flight_accel_bias_context_valid =
        (g_arm_state == ARM_STATE_ARMED) &&
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) &&
        (position_controller.phase == POSITION_CONTROL_PHASE_HOLD) &&
        (s_nav_last.gps_velocity_valid != 0U) &&
        (s_nav_last.gps_velocity_control_ready != 0U) &&
        (s_nav_last.gps_position_velocity_valid != 0U) &&
        (gps_velocity_candidate_disagreement_mps <= 
            POSITION_BRAKE_GPS_VELOCITY_DISAGREEMENT_MAX_MPS);

    /*
     * 即使刚满足 Bias 修正环境，也继续等待一段稳定时间，
     * 避免刚进入 HOLD 时的控制瞬态污染 Bias Observer。
     */
    if (flight_accel_bias_context_valid)
    {
        if (s_flight_accel_bias_hold_time_s <
            FLIGHT_ACCEL_BIAS_SETTLE_TIME_S)
        {
            s_flight_accel_bias_hold_time_s += dt;

            if (s_flight_accel_bias_hold_time_s >
                FLIGHT_ACCEL_BIAS_SETTLE_TIME_S)
            {
                s_flight_accel_bias_hold_time_s =
                    FLIGHT_ACCEL_BIAS_SETTLE_TIME_S;
            }
        }
    }
    else
    {
        s_flight_accel_bias_hold_time_s = 0.0f;
    }

    const bool allow_flight_accel_bias_correction =
        flight_accel_bias_context_valid &&
        (s_flight_accel_bias_hold_time_s >=
         FLIGHT_ACCEL_BIAS_SETTLE_TIME_S);

    /*
     * GPS 状态突变造成的大 Innovation 只允许在 Manual 中重新对齐 Estimator，
     * 辅助模式已经接管时禁止突然重建状态。
     */
    const bool allow_estimator_state_reacquire =
        (s_horizontal_mode_active == HORIZONTAL_MODE_MANUAL);

    /*
     * GPS Velocity 只处理新的 RMC Sequence，
     * 防止同一观测被重复融合。
     */
    if (s_nav_last.rmc_sequence != horizontal_estimator.last_gps_sequence)
    {
        const bool require_rmc_position_window_v_confirmation =
            (s_position_hold_requested != 0U) ||
            (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD);

        const bool rmc_vector_use_allowed =
            FlightControl_UpdateRmcVectorUseAllowed(
                require_rmc_position_window_v_confirmation);

        (void)HorizontalEstimator_CorrectGps(
            s_nav_last.gps_velocity_n_mps,
            s_nav_last.gps_velocity_e_mps,
            s_nav_last.rmc_sequence,
            s_nav_last.rmc_last_update_ms,
            navigation_yaw_rad,
            s_nav_last.gps_velocity_valid != 0U,
            rmc_vector_use_allowed,
            allow_flight_accel_bias_correction,
            allow_estimator_state_reacquire);
    }

/* ----------------------------- GPS Position 失效后的局部参考系管理 ---------------------------- */

    /*
     * Position Quality 失效后清除局部参考系。
     * GPS 恢复时从当前位置重新建立 Reference，
     * 避免长时间失锁前的旧位置状态产生巨大 Innovation。
     */
    if (s_nav_last.gps_position_control_ready == 0U)
    {
        if (horizontal_estimator.position_initialized)
            HorizontalEstimator_ResetPosition();
    }
    else if (!horizontal_estimator.position_initialized)
    {
        (void)HorizontalEstimator_SetPositionReference(
            s_nav_last.gps_lat,
            s_nav_last.gps_lon,
            s_nav_last.rmc_sequence);
    }
    else if (s_nav_last.rmc_sequence !=
             horizontal_estimator.last_position_sequence)
    {
        (void)HorizontalEstimator_CorrectPosition(
            s_nav_last.gps_lat,
            s_nav_last.gps_lon,
            s_nav_last.rmc_sequence,
            true,
            allow_estimator_state_reacquire);
    }

    HorizontalEstimator_UpdateHealth(HAL_GetTick());
}

/* 根据三段 SC 开关位置解析用户请求的水平控制模式。 */
static FlightHorizontalMode_t FlightControl_GetRequestedHorizontalMode(void)
{
    const uint16_t sc = s_rc_last.channels[HORIZONTAL_MODE_RC_CHANNEL];

    if (sc >= HORIZONTAL_MODE_SC_HIGH_MIN)
        return HORIZONTAL_MODE_POSITION_HOLD;

    if (sc >= HORIZONTAL_MODE_SC_MIDDLE_MIN &&
        sc <= HORIZONTAL_MODE_SC_MIDDLE_MAX)
    {
        return HORIZONTAL_MODE_VELOCITY_HOLD;
    }

    return HORIZONTAL_MODE_MANUAL;
}

/* 同步用于状态查询和 Blackbox 的 Horizontal Mode 请求/接管标志。 */
static void FlightControl_RefreshHorizontalModeFlags(FlightHorizontalMode_t requested)
{
    s_velocity_hold_requested =
        (requested == HORIZONTAL_MODE_VELOCITY_HOLD) 
            ? 1U 
            : 0U;
            
    s_position_hold_requested =
        (requested == HORIZONTAL_MODE_POSITION_HOLD) 
            ? 1U 
            : 0U;

    s_velocity_hold_active =
        (s_horizontal_mode_active == HORIZONTAL_MODE_VELOCITY_HOLD) 
            ? 1U 
            : 0U;

    s_position_hold_active =
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) 
            ? 1U 
            : 0U;
}

/*
 * 退出所有水平辅助控制并返回 Manual。
 * 
 * 清除 Position / Velocity Controller 动态状态。
 * 同时清除 Pilot Command 与接管 Blend 状态。
 */
static void FlightControl_ExitHorizontalMode(void)
{
    VelocityController_Reset();
    PositionController_Reset();

    s_horizontal_mode_active = HORIZONTAL_MODE_MANUAL;
    s_velocity_target_n_mps = 0.0f;
    s_velocity_target_e_mps = 0.0f;

    s_pilot_velocity_cmd_n_mps = 0.0f;
    s_pilot_velocity_cmd_e_mps = 0.0f;

    s_horizontal_control_blend = 0.0f;
}

/*
 * 检查 Estimator Velocity 与当前 GPS Velocity observation 是否足够一致，
 * 防止状态估计已经明显偏离时突然切入水平闭环控制。
 */
static bool FlightControl_HorizontalVelocityEntryConsistent(void)
{
    if (!horizontal_estimator.initialized ||
        !horizontal_estimator.last_gps_accepted ||
        s_nav_last.gps_velocity_valid == 0U)
    {
        return false;
    }

    const float rmc_speed_mps = sqrtf(
        s_nav_last.gps_velocity_n_mps *
            s_nav_last.gps_velocity_n_mps +
        s_nav_last.gps_velocity_e_mps *
            s_nav_last.gps_velocity_e_mps);

    if (!(rmc_speed_mps >= 0.0f && rmc_speed_mps <= 30.0f))
        return false;

    /*
     * 低速时不信任 RMC Course，因此只使用零速作为观测参考；
     * 高于 Course 使用门限后才比较完整 N/E Velocity Vector。
     */
    const bool rmc_course_usable =
        rmc_speed_mps >= NAV_RMC_VECTOR_MIN_SPEED_MPS;

    const float observed_velocity_n_mps =
        rmc_course_usable
            ? s_nav_last.gps_velocity_n_mps
            : 0.0f;

    const float observed_velocity_e_mps =
        rmc_course_usable
            ? s_nav_last.gps_velocity_e_mps
            : 0.0f;

    const float error_n_mps =
        observed_velocity_n_mps -
        horizontal_estimator.velocity_n_mps;

    const float error_e_mps =
        observed_velocity_e_mps -
        horizontal_estimator.velocity_e_mps;

    const float error_mps = sqrtf(
        error_n_mps * error_n_mps +
        error_e_mps * error_e_mps);

    return error_mps >= 0.0f &&
           error_mps <=
               HORIZONTAL_MODE_ENTRY_MAX_VELOCITY_ERROR_MPS;
}

/*
 * 检查 Estimator Position 与最新 GPS Local Position 是否足够一致，
 * 作为 Position Hold 接管前的状态连续性门限。
 */
static bool FlightControl_HorizontalPositionEntryConsistent(void)
{
    if (!horizontal_estimator.position_initialized ||
        !horizontal_estimator.last_position_accepted ||
        horizontal_estimator.last_position_rejected)
    {
        return false;
    }

    const float error_n_m =
        horizontal_estimator.gps_position_n_m -
        horizontal_estimator.position_n_m;

    const float error_e_m =
        horizontal_estimator.gps_position_e_m -
        horizontal_estimator.position_e_m;

    const float error_m = sqrtf(
        error_n_m * error_n_m +
        error_e_m * error_e_m);

    return error_m >= 0.0f &&
           error_m <= HORIZONTAL_MODE_ENTRY_MAX_POSITION_ERROR_M;
}

/*
 * 尝试进入指定水平辅助模式。
 * 
 * 从 Manual 进入时清除 Velocity Controller 历史状态；
 * Velocity Hold 与 Position Hold 直接切换时保留 Velocity Integral，
 * 避免已经建立的静态扰动补偿在模式切换时突然消失。
 */
static bool FlightControl_TryEnterHorizontalMode(
    FlightHorizontalMode_t requested, 
    bool estimator_healthy)
{
    if (requested == HORIZONTAL_MODE_MANUAL)
        return false;

    if ((requested == HORIZONTAL_MODE_VELOCITY_HOLD &&
         VELOCITY_HOLD_CONTROL_ENABLE == 0U) ||
        (requested == HORIZONTAL_MODE_POSITION_HOLD &&
         POSITION_HOLD_CONTROL_ENABLE == 0U))
    {
        return false;
    }

    /*
     * 两种水平辅助模式共同要求：
     * GPS Velocity 可用于控制，Estimator 健康，连续接受足够样本，
     * Estimator 与 GPS Velocity 一致，且当前油门已经进入飞行工作区。
     */
    const bool common_entry_ready =
        (s_nav_last.gps_velocity_control_ready != 0U) &&
        estimator_healthy &&
        (horizontal_estimator.gps_accept_streak >=
         HORIZONTAL_MODE_ENTRY_GPS_ACCEPT_STREAK) &&
        FlightControl_HorizontalVelocityEntryConsistent() &&
        fc.throttle >= HORIZONTAL_MODE_ENTRY_MIN_THROTTLE;

    if (!common_entry_ready)
        return false;

    const bool entering_from_manual =
        (s_horizontal_mode_active == HORIZONTAL_MODE_MANUAL);

    /*
     * 从 Manual 接管时，让 Pilot Velocity Command 从当前估计速度开始，
     * 避免目标速度瞬间从零跳变。
     */
    if (entering_from_manual)
    {
        s_pilot_velocity_cmd_n_mps =
            horizontal_estimator.velocity_n_mps;

        s_pilot_velocity_cmd_e_mps =
            horizontal_estimator.velocity_e_mps;

        s_horizontal_control_blend = 0.0f;
    }

    if (requested == HORIZONTAL_MODE_VELOCITY_HOLD)
    {
        /*
         * Manual -> Velocity 时从干净的 Velocity Controller 状态开始；
         * Position -> Velocity 时保留已有 Integral 和 控制状态。
         */
        if (entering_from_manual)
            VelocityController_Reset();

        PositionController_Reset();
        s_horizontal_mode_active = HORIZONTAL_MODE_VELOCITY_HOLD;
        return true;
    }

    if (requested == HORIZONTAL_MODE_POSITION_HOLD)
    {
        /*
         * Position Hold 比 Velocity Hold 额外要求可靠的 Position 状态，
         * 并要求连续多帧 Position Observation 已通过融合门限。
         */
        if (s_nav_last.gps_position_control_ready == 0U ||
            !horizontal_estimator.position_initialized ||
            !horizontal_estimator.last_position_accepted ||
            horizontal_estimator.last_position_rejected ||
            horizontal_estimator.position_accept_streak <
                HORIZONTAL_MODE_ENTRY_POSITION_ACCEPT_STREAK ||
            !FlightControl_HorizontalPositionEntryConsistent())
        {
            return false;
        }

        if (!PositionController_Enter(&horizontal_estimator))
        {
            return false;
        }

        /*
         * Manual -> Position 时从干净的 Velocity Controller 状态开始；
         * Velocity -> Position 时保留内层速度环已有的静态扰动补偿。
         */
        if (entering_from_manual)
            VelocityController_Reset();

        s_horizontal_mode_active = HORIZONTAL_MODE_POSITION_HOLD;
        return true;
    }

    return false;
}

/*
 * 对水平摇杆应用 Deadzone 与 Expo。
 * 
 * Deadzone 消除中杆附近的小幅输入偏置；
 * Expo 降低中杆区域灵敏度，而满杆仍保持原最大输入幅值。
 */
static float FlightControl_ApplyHorizontalPilotDeadzone(float input_deg)
{
    const float magnitude = fabsf(input_deg);

    if (magnitude <= HORIZONTAL_PILOT_DEADZONE_DEG)
        return 0.0f;

    float normalized =
        (magnitude - HORIZONTAL_PILOT_DEADZONE_DEG) /
        (HORIZONTAL_PILOT_INPUT_MAX_DEG - HORIZONTAL_PILOT_DEADZONE_DEG);

    normalized = fminf(normalized, 1.0f);

    /*
     * 应用线性插值：
     * y = (1 - e) * x + e * x^3
     */
    const float curved =
        (1.0f - HORIZONTAL_PILOT_EXPO) * normalized +
        HORIZONTAL_PILOT_EXPO * normalized * normalized * normalized;

    const float scaled = curved * HORIZONTAL_PILOT_INPUT_MAX_DEG;

    return (input_deg < 0.0f) ? -scaled : scaled;
}

/*
 * 将 Roll/Pitch 摇杆映射为机体系 Forward/Right Velocity Command，
 * 再根据当前 Yaw 旋转到 Navigation North/East 坐标系。
 */
static void FlightControl_GetPilotVelocityNe(float max_speed_mps,
                                             float yaw_rad,
                                             float *velocity_n_mps,
                                             float *velocity_e_mps)
{
    const float pitch_input_deg =
        FlightControl_ApplyHorizontalPilotDeadzone(
            Map_Pitch(s_rc_last.channels[1]));

    const float roll_input_deg =
        FlightControl_ApplyHorizontalPilotDeadzone(
            Map_Roll(s_rc_last.channels[0]));

    const float velocity_forward_mps =
        -pitch_input_deg / HORIZONTAL_PILOT_INPUT_MAX_DEG * max_speed_mps;

    const float velocity_right_mps =
        roll_input_deg / HORIZONTAL_PILOT_INPUT_MAX_DEG * max_speed_mps;

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    *velocity_n_mps =
        cy * velocity_forward_mps - sy * velocity_right_mps;

    *velocity_e_mps =
        sy * velocity_forward_mps + cy * velocity_right_mps;
}

/*
 * 对 Pilot Velocity Command 执行二维 Slew Rate Limit。
 * 
 * 这里只限制飞手输入目标的变化速度，不限制 Controller Feedback，
 * 因此外部扰动仍可立即获得完整反馈控制权限。
 */
static void FlightControl_UpdatePilotVelocityCommand(float target_n_mps,
                                                     float target_e_mps,
                                                     float dt,
                                                     float *command_n_mps,
                                                     float *command_e_mps)
{
    if (command_n_mps == NULL || command_e_mps == NULL)
        return;

    const float delta_n_mps = target_n_mps - *command_n_mps;
    const float delta_e_mps = target_e_mps - *command_e_mps;

    const float delta_mps = sqrtf(
        delta_n_mps * delta_n_mps +
        delta_e_mps * delta_e_mps);

    const float max_step_mps =
        HORIZONTAL_PILOT_ACCEL_LIMIT_MPS2 * dt;

    /*
     * 当二维目标变化超过本周期允许步长时，
     * 按原方向同比例缩放 N/E 增量。
     */
    if (delta_mps > max_step_mps && delta_mps > 0.0001f)
    {
        const float scale = max_step_mps / delta_mps;
        *command_n_mps += delta_n_mps * scale;
        *command_e_mps += delta_e_mps * scale;
    }
    else
    {
        *command_n_mps = target_n_mps;
        *command_e_mps = target_e_mps;
    }
}

/*
 * 更新水平辅助模式状态机并生成 Navigation Roll/Pitch Target。
 * 
 * 模式因 GPS，Estimator 或其他安全条件异常退出后，
 * 必须重新观察到 Manual 档及新的辅助模式切换沿，
 * 防止条件恢复后控制器在飞手未重新确认的情况下自动接管。
 */
static void FlightControl_UpdateHorizontalMode(float dt)
{
    const FlightHorizontalMode_t requested =
        FlightControl_GetRequestedHorizontalMode();

    const bool request_changed =
        requested != s_horizontal_mode_last_request;

    s_horizontal_mode_last_request = requested;

    const bool estimator_healthy =
        HorizontalEstimator_IsHealthy(HAL_GetTick());

    /*
     * 只有真实观察到一次 SC Manual 档，
     * 才允许之后的新辅助模式请求触发接管。
     */
    if (requested == HORIZONTAL_MODE_MANUAL)
    {
        s_horizontal_manual_seen = 1U;

        if (s_horizontal_mode_active != HORIZONTAL_MODE_MANUAL)
            FlightControl_ExitHorizontalMode();

        FlightControl_RefreshHorizontalModeFlags(requested);
        return;
    }

    /*
     * 已经处于辅助模式时，持续检查维持控制所需的基础条件。
     */
    if (s_horizontal_mode_active != HORIZONTAL_MODE_MANUAL)
    {
        bool maintain_ready =
            estimator_healthy &&
            fc.throttle >= HORIZONTAL_MODE_ENTRY_MIN_THROTTLE;

        if (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD)
        {
            maintain_ready =
                maintain_ready &&
                (s_nav_last.gps_position_control_ready != 0U) &&
                horizontal_estimator.position_initialized &&
                (horizontal_estimator.position_accept_streak >=
                 HORIZONTAL_MODE_ENTRY_POSITION_ACCEPT_STREAK) &&
                (horizontal_estimator.last_position_rejected == 0U);
        }

        if (!maintain_ready)
        {
            /*
             * GPS，Estimator 或油门条件失效后立即退出辅助模式。
             * 条件恢复也不能自动重新接管，必须由飞手重新经过 Manual。
             */
            FlightControl_ExitHorizontalMode();
            s_horizontal_manual_seen = 0U;

            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }

        if (requested != s_horizontal_mode_active)
        {
            /*
             * 三段 SC 从低档拨向高档时可能短暂经过中档，
             * 因此允许 Velocity Hold 与 Position Hold 直接切换。
             */
            if (!FlightControl_TryEnterHorizontalMode(requested, estimator_healthy))
            {
                FlightControl_ExitHorizontalMode();
                s_horizontal_manual_seen = 0U;

                FlightControl_RefreshHorizontalModeFlags(requested);
                return;
            }
        }
    }
    else
    {
        /*
         * Manual -> 辅助模式必须同时满足：
         * 1. 已经真实观察到 Manual 档；
         * 2. 当前出现新的模式切换沿。
         */
        if (!request_changed || !s_horizontal_manual_seen)
        {
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }

        /*
         * 一次切换沿只允许尝试一次。
         * 即使条件不足导致进入失败，也必须重新回到 Manual 后才能重试。
         */
        s_horizontal_manual_seen = 0U;

        if (!FlightControl_TryEnterHorizontalMode(requested, estimator_healthy))
        {
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }
    }

    /*
     * Pilot Velocity 使用 True-North Navigation Yaw，
     * 与 Horizontal Estimator 的 N/E 坐标定义保持一致。
     */
    const float yaw_rad =
        (fc.yaw_meas + NAV_MAG_DECLINATION_DEG) * 0.0174532925f;

    float pilot_velocity_target_n_mps;
    float pilot_velocity_target_e_mps;

    const float pilot_speed_limit_mps =
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) 
            ? POSITION_HOLD_PILOT_SPEED_MPS 
            : VELOCITY_HOLD_MAX_SPEED_MPS;

    FlightControl_GetPilotVelocityNe(
        pilot_speed_limit_mps,
        yaw_rad,
        &pilot_velocity_target_n_mps,
        &pilot_velocity_target_e_mps);

    FlightControl_UpdatePilotVelocityCommand(
        pilot_velocity_target_n_mps,
        pilot_velocity_target_e_mps,
        dt,
        &s_pilot_velocity_cmd_n_mps,
        &s_pilot_velocity_cmd_e_mps);

    /*
     * Position BRAKING 阶段同时观察 RMC Velocity
     * 与 Position-window Velocity。
     */
    const float brake_velocity_disagreement_n_mps =
        s_nav_last.rmc_velocity_n_mps -
        s_nav_last.gps_position_velocity_n_mps;

    const float brake_velocity_disagreement_e_mps =
        s_nav_last.rmc_velocity_e_mps -
        s_nav_last.gps_position_velocity_e_mps;

    const float brake_velocity_disagreement_mps = sqrtf(
        brake_velocity_disagreement_n_mps *
            brake_velocity_disagreement_n_mps +
        brake_velocity_disagreement_e_mps *
            brake_velocity_disagreement_e_mps);

    const float brake_rmc_speed_mps = sqrtf(
        s_nav_last.rmc_velocity_n_mps *
            s_nav_last.rmc_velocity_n_mps +
        s_nav_last.rmc_velocity_e_mps *
            s_nav_last.rmc_velocity_e_mps);

    const float brake_position_window_speed_mps = sqrtf(
        s_nav_last.gps_position_velocity_n_mps *
            s_nav_last.gps_position_velocity_n_mps +
        s_nav_last.gps_position_velocity_e_mps *
            s_nav_last.gps_position_velocity_e_mps);

    /*
     * 使用两种 GPS Velocity Observation 中较大的速度模长作为
     * BRAKING -> HOLD 捕获判断，采用较保守的“仍在运动”判据。
     */
    const float brake_observed_speed_mps =
        (brake_rmc_speed_mps > brake_position_window_speed_mps)
            ? brake_rmc_speed_mps
            : brake_position_window_speed_mps;

    const bool brake_velocity_observations_consistent =
        (s_nav_last.gps_velocity_valid != 0U) &&
        (s_nav_last.gps_position_velocity_valid != 0U) &&
        (brake_velocity_disagreement_mps <=
         POSITION_BRAKE_GPS_VELOCITY_DISAGREEMENT_MAX_MPS);

    if (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD)
    {
        if (!PositionController_Update(
                &horizontal_estimator,
                s_pilot_velocity_cmd_n_mps,
                s_pilot_velocity_cmd_e_mps,
                brake_velocity_observations_consistent,
                brake_observed_speed_mps,
                dt))
        {
            FlightControl_ExitHorizontalMode();
            s_horizontal_manual_seen = 0U;
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }

        s_velocity_target_n_mps =
            position_controller.velocity_target_n_mps;
        s_velocity_target_e_mps =
            position_controller.velocity_target_e_mps;
    }
    else
    {
        s_velocity_target_n_mps = s_pilot_velocity_cmd_n_mps;
        s_velocity_target_e_mps = s_pilot_velocity_cmd_e_mps;
    }

    /*
     * Velocity Integral 学习权限；
     * 
     * Velocity Hold：正常学习；
     * Position Hold BRAKING/MOVING：冻结；
     * Position Hold HOLD：
     *      条件稳定时正常学习，否则只允许已有 Integral 卸载。
     */
    VelocityIntegralMode_t velocity_integral_mode =
        VELOCITY_INTEGRAL_MODE_FROZEN;

    if(s_horizontal_mode_active == HORIZONTAL_MODE_VELOCITY_HOLD)
    {
        velocity_integral_mode = VELOCITY_INTEGRAL_MODE_LEARN;
    }
    else if(s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD)
    {
        if(position_controller.phase == POSITION_CONTROL_PHASE_HOLD)
        {
            velocity_integral_mode =
                (position_controller.integral_learning_allowed != 0U)
                    ? VELOCITY_INTEGRAL_MODE_LEARN
                    : VELOCITY_INTEGRAL_MODE_UNLOAD_ONLY;
        }
        else
        {
            velocity_integral_mode = VELOCITY_INTEGRAL_MODE_FROZEN;
        }
    }

    VelocityController_Update(
        s_velocity_target_n_mps,
        s_velocity_target_e_mps,
        horizontal_estimator.velocity_n_mps,
        horizontal_estimator.velocity_e_mps,
        horizontal_estimator.accel_n_mps2,
        horizontal_estimator.accel_e_mps2,
        yaw_rad,
        velocity_integral_mode,
        dt);

    FlightControl_RefreshHorizontalModeFlags(requested);

    /*
     * 辅助模式刚接管时逐渐混入 Navigation Roll/Pitch Target，
     * 避免 Manual Target 与闭环 Target 不连续产生姿态阶跃。
     */
    if (s_horizontal_control_blend < 1.0f)
    {
        s_horizontal_control_blend +=
            dt / HORIZONTAL_MODE_BLEND_TIME_S;
        if (s_horizontal_control_blend > 1.0f)
            s_horizontal_control_blend = 1.0f;
    }

    fc.roll_target += s_horizontal_control_blend *
                      (velocity_controller.roll_target_deg - fc.roll_target);
    fc.pitch_target += s_horizontal_control_blend *
                       (velocity_controller.pitch_target_deg - fc.pitch_target);
}

/*
 * 执行一次姿态控制计算。
 * 
 * 调用顺序固定为：
 * RC Mapping -> Airmode -> Horizontal Mode -> Yaw Mode ->
 * Angle Controller -> Rate Controller。
 */
static void FlightControl_Update(float dt, float gx, float gy, float gz)
{
    fc.roll_target = Map_Roll(s_rc_last.channels[0]);
    fc.pitch_target = Map_Pitch(s_rc_last.channels[1]);
    fc.throttle = Map_Throttle(s_rc_last.channels[2]);

    // Disarmed 状态不运行姿态控制器，避免产生任何非零 Motor Command。
    if (g_arm_state != ARM_STATE_ARMED)
    {
        fc.airmode_active = 0U;
        FlightControl_Reset();
        return;
    }

    /* 
     * 测试阶段的Airmode状态机。
     *
     * 未启用时，油门达到激活阈值才开始运行姿态PID。
     * 已启用后，油门降到退出阈值以下便停止PID并清空控制状态。
     *
     * 注意：这是近地调试策略。正式飞行时，空中低油门仍需姿态控制，
     * 后续应由airborne/landed状态决定是否退出Airmode。
     * */
    if (!fc.airmode_active)
    {
        if (fc.throttle < AIRMODE_ACTIVATION_THROTTLE)
        {
            FlightControl_Reset();
            return;
        }

        fc.airmode_active = 1U;
    }
    else if (fc.throttle < AIRMODE_DEACTIVATION_THROTTLE)
    {
        fc.airmode_active = 0U;
        FlightControl_Reset();
        return;
    }

    FlightControl_UpdateHorizontalMode(dt);

    // SA 对应 Manual Yaw Rate / Heading Hold 模式选择。
    if (s_rc_last.channels[5] > 1500)
    {
        fc.yaw_mode = 1U;
        fc.yaw_rate_target = Map_Yaw(s_rc_last.channels[3]);
        if (fabsf(fc.yaw_rate_target) < 5.0f)
            fc.yaw_rate_target = 0.0f;
    }
    else
    {
        /*
         * Manual Yaw -> Heading Hold 时，以当前航向作为新的锚定点，
         * 并清除两级 Yaw Controller 的历史状态，
         * 避免旧 Integral / Derivative 在模式切换后产生瞬态输出。
         */
        if (fc.yaw_mode != 0U)
        {
            fc.yaw_target = fc.yaw_meas;
            
            PID_Reset(&pid_yaw);

            rate_controller.yaw.integral = 0.0f;
            PID_ResetDerivativeOnly(&rate_controller.yaw);
        }

        fc.yaw_mode = 0;

        fc.yaw_rate_target = YawHeadingHold_Update(
            &pid_yaw, fc.yaw_target, fc.yaw_meas, dt);
    }

    /* Angle Controller：姿态角外环。 */
    AngleController_Update(fc.roll_target, fc.pitch_target,
                           fc.roll_meas, fc.pitch_meas, dt);

    fc.roll_rate_target = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /*
     * 当前仅在 Position Hold 中启用 Roll/Pitch Rate Feedforward，
     * 保持 Manual 与 Velocity Hold 原有控制相应不变。
     */
    RateController_SetRollPitchFeedForwardEnabled(
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) ? 1U : 0U);

    /* Rate Controller：角速度内环。 */
    RateController_Update(fc.roll_rate_target,
                          fc.pitch_rate_target,
                          fc.yaw_rate_target,
                          gx, gy, gz, dt);

    fc.roll_cmd = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd = rate_controller.yaw_output;
}

/* 将浮点量按指定比例缩放并饱和转换为 int16_t。 */
static int16_t BB_ToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
        return 32767;
    if (scaled < -32768.0f)
        return -32768;

    return (int16_t)scaled;
}

/* 将浮点量按比例缩放，四舍五入饱和转换为 uint16_t。 */
static uint16_t BB_ToUInt16(float value, float scale)
{
    const float scaled = value * scale;

    if (scaled <= 0.0f)
        return 0U;
    if (scaled >= 65535.0f)
        return 65535U;

    return (uint16_t)(scaled + 0.5f);
}

/*
 * Flight Control 主任务。
 * 
 * 循环由双 IMU DRDY EventFlags 驱动，不使用 osDelay 节流。
 * 只有 Active IMU 产生新的有效时间戳时才运行姿态估计与控制器；
 * Standby Refresh，Timeout，Dual Fault 与 Timing Anomaly
 * 均在进入控制算法前单独处理。
 */
void App_FlightCtrl_Task(void *argument)
{
    (void)argument;

    /*
     * 初始化双 IMU，冗余管理及完整控制算法链。
     * fail_mask 记录各 IMU 初始化失败状态，并交由 Redundancy 模块管理。
     */
    uint8_t fail_mask = ICM_InitAll();

    ImuRedundancy_Init(fail_mask);
    FlightControl_Init();
    Arm_Init();

    /* 连续 Active IMU Timing Anomaly 计数。 */
    static uint8_t timing_anomaly_streak = 0U;

    IcmData_t active_data;
    float dt;

    uint16_t m1, m2, m3, m4;

    bool attitude_initialized = false;

    for (;;)
    {
        uint8_t fresh_imu_flags = 0U;

        /*
         * WaitAny 可被任意一颗 IMU 唤醒。
         * 返回状态区分 Active 新帧，Standby Refresh，Timeout，
         * Calibration 与 Timing Anomaly，
         * 防止同一份 Active IMU 数据被重复送入姿态估计和PID。
         */
        ImuUpdateResult_t imu_result = 
            ImuRedundancy_Update(&active_data, &dt, &fresh_imu_flags);

        g_heartbeat.flightctrl_last_tick = osKernelGetTickCount();

        // 双 IMU 未同时故障时认为 IMU Health 满足 ARM Ready 条件。
        if (g_imu_health.dual_fault)
        {
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);
        }
        else
        {
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);
        }

        // Gyro Calibration 状态同步到 SystemReady EventFlags。
        if (ImuCalibration_IsReady())
        {
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);
        }
        else
        {
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);
        }

        if (imu_result == IMU_UPDATE_CALIBRATION)
        {
            /*
             * IMU Calibration 期间禁止任何控制输出；
             * 机体运动只会使静止校准条件继续延后。
             */
            FlightControl_Reset();
            BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            continue;
        }

        if (imu_result == IMU_UPDATE_TIMING_ANOMALY)
        {
            /*
             * 当前帧时间基准异常时不执行姿态积分和 PID，
             * 并重建所有 Derivative 历史采样基准，
             * 避免跨越异常 Dt 计算出 Derivative Spike。
             */
            PID_ResetDerivativeOnly(&angle_controller.roll);
            PID_ResetDerivativeOnly(&angle_controller.pitch);

            PID_ResetDerivativeOnly(&rate_controller.roll);
            PID_ResetDerivativeOnly(&rate_controller.pitch);
            PID_ResetDerivativeOnly(&rate_controller.yaw);

            PID_ResetDerivativeOnly(&pid_yaw);

            if (timing_anomaly_streak < 0xFFU)
                timing_anomaly_streak++;

            /*
             * 单次 Timing Anomaly 暂时保持上一帧 Motor Output；
             * 连续异常达到门限后认为实时控制链已经不可信，
             * 强制 Disarm 并发送 DShot 0。
             */
            if (timing_anomaly_streak >= 3U)
            {
                Arm_ForceDisarm();
                BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            }

            continue;
        }

        if (imu_result == IMU_UPDATE_STANDBY_ONLY)
        {
            /*
             * Standby IMU 的独立刷新不能推进控制周期，
             * 保持上一帧 Motor Output，等待新的 Active IMU 数据。
             */
            continue;
        }

        if (imu_result == IMU_UPDATE_ACTIVE_FRAME)
        {
            // 只有新的有效 Active Frame 才清除连续 Timing Anomaly 计数。
            timing_anomaly_streak = 0U;
        }

        if ((imu_result == IMU_UPDATE_TIMEOUT) || (imu_result == IMU_UPDATE_DUAL_FAULT))
        {
            if ((imu_result == IMU_UPDATE_DUAL_FAULT) || g_imu_health.dual_fault)
            {
                Arm_ForceDisarm();
            }

            if (g_arm_state != ARM_STATE_ARMED)

            {
                BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            }

            continue;
        }

        /*
         * 到达此处必然是时间有效的 Active IMU 新帧。
         * dt 来自 Active IMU 连续两次实际采样时间戳，
         * 而不是 RTOS Task 两次唤醒之间的时间。
         */
        Attitude_ComputeAccelAngles(&active_data, &attitude);

        /*
         * Level Trim 只修正 Gravity Observation，
         * 不直接修正融合姿态状态或 Gyro Measurement。
         */
        LevelTrim_Apply((IcmInstance_t)g_imu_health.active_imu_sel,
                        &attitude.accel_roll,
                        &attitude.accel_pitch);

        if (!attitude_initialized)
        {
            /*
             * 首帧直接使用 Accel Gravity Observation 初始化 Roll/Pitch，
             * 避免从零角度缓慢收敛产生虚假姿态误差。
             */
            attitude.roll = attitude.accel_roll;
            attitude.pitch = attitude.accel_pitch;
            attitude_initialized = true;
        }
        else
        {
            /*
             * Armed 飞行期间降低 Accel 修正带宽，
             * 减小水平运动 Acceleration 对 Roll/Pitch 污染；
             * Disarmed 时恢复较快重力修正，使姿态重新收敛。
             */
            Attitude_Update(
                &active_data,
                &attitude,
                dt,
                g_arm_state == ARM_STATE_ARMED);
        }

        fc.roll_meas = attitude.roll * 57.29578f;
        fc.pitch_meas = attitude.pitch * 57.29578f;

        /*
         * 每个有效 Active IMU 周期均由 Gyro_Z 推进 Yaw；
         * Mag 只在新的低频样本到达时执行长期漂移修正。
         */
        YawEstimator_UpdateGyro(active_data.gz, dt);

        MagData_t mag;

        if (osMessageQueueGet(MagDataMailboxHandle, &mag, NULL, 0) == osOK)
        {
            (void)YawEstimator_CorrectMag(
                &mag,
                attitude.roll,
                attitude.pitch,
                g_arm_state == ARM_STATE_DISARMED);
        }

        if (YawEstimator_IsInitialized())
        {
            attitude.yaw = YawEstimator_GetYawRad();

            fc.yaw_meas = attitude.yaw * 57.29578f;

            /*
             * Disarmed 时持续让 Heading Hold Target 跟随当前航向，
             * 避免解锁瞬间产生旧目标航向误差。
             */
            if (g_arm_state == ARM_STATE_DISARMED)
            {
                fc.yaw_target = fc.yaw_meas;
            }
        }

        FlightControl_UpdateHorizontalEstimator(&active_data, dt);

        /*
         * 非阻塞获取最新 RC，
         * 并处理依赖 RC 开关输入的 Mag Calibration 与 Level Trim。
         */
        osMessageQueueGet(RCChannelMailboxHandle, &s_rc_last, NULL, 0);

        MagCalibration_HandleRc(&s_rc_last);
        LevelTrim_HandleRc(&s_rc_last);

        Arm_Update(&s_rc_last, fc.roll_meas, fc.pitch_meas, dt);

        /* 
         * Arm_Update 可能因为人工 Motor Kill、RC lost、低压、
         * 双 IMU 故障或倾倒检测而撤销解锁。
         *
         * 一旦撤销，本周期立即发送 DShot 0，
         * 不再进入 PID、Mixer 和正常电机输出。
         */
        if (g_arm_state != ARM_STATE_ARMED)
        {
            FlightControl_Reset();
            BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            continue;
        }

        FlightControl_Update(dt, active_data.gx, active_data.gy, active_data.gz);

        /*
         * Power Monitor 输出当前允许使用的 Throttle 比例，
         * 正常范围为 600~1000 permille。
         * 
         * 越界视为尚未初始化或状态损坏，回退到 1000，
         * 避免异常状态错误地将飞行油门削减至零。
         */
        uint16_t current_limit = g_power_health.current_limit_permille;

        if (current_limit < 600U || current_limit > 1000U)
        {
            current_limit = 1000U;
        }

        /*
         * Current Limit 只缩放 Collective Throttle，
         * Mixer 仍保留差动姿态控制权限。
         * 使用 uint32_t 完成中间算法，避免 uint16_t 溢出。
         */
        uint16_t limited_throttle = 
            (uint16_t)(((uint32_t)fc.throttle * 
                        current_limit) / 
                        1000U);

        Mixer(
            fc.roll_cmd,
            fc.pitch_cmd,
            fc.yaw_cmd,
            limited_throttle,
            fc.airmode_active,
            &m1,
            &m2,
            &m3,
            &m4);

#if NAV_SHADOW_FORCE_DSHOT_ZERO

        BSP_DSHOT_Send(
            0U,
            0U,
            0U,
            0U);

#else

        BSP_DSHOT_Send(
            (uint16_t)(m1 + 48U),
            (uint16_t)(m2 + 48U),
            (uint16_t)(m3 + 48U),
            (uint16_t)(m4 + 48U));

#endif
        /*
         * Control Log：
         * 控制环标称 800 Hz，按 4 分频记录为 200 Hz。
         * 
         * 所有字段均取自当前控制周期，
         * 避免跨 Task 读取 fc 导致数据断裂。
         * BB_LogControl() 只写入 RAM Buffer，不执行 SD I/O。
         */
#define BB_LOG_DIVIDER 4U
        static uint8_t s_bb_log_divider = 0U;

        s_bb_log_divider++;

        if (s_bb_log_divider >= BB_LOG_DIVIDER)
        {
            s_bb_log_divider = 0U;

            BB_ControlData_t log = {0};

            log.timestamp_cycle =
                active_data.timestamp_cycle;

            log.angle_cdeg[0] =
                BB_ToInt16(
                    fc.roll_meas,
                    100.0f);

            log.angle_cdeg[1] =
                BB_ToInt16(
                    fc.pitch_meas,
                    100.0f);

            log.angle_cdeg[2] =
                BB_ToInt16(
                    fc.yaw_meas,
                    100.0f);

            log.angle_target_cdeg[0] =
                BB_ToInt16(
                    fc.roll_target,
                    100.0f);

            log.angle_target_cdeg[1] =
                BB_ToInt16(
                    fc.pitch_target,
                    100.0f);

            log.angle_target_cdeg[2] =
                BB_ToInt16(
                    fc.yaw_target,
                    100.0f);

            float trim_roll_rad;
            float trim_pitch_rad;

            if (LevelTrim_GetOffsets(
                    (IcmInstance_t)
                        g_imu_health.active_imu_sel,
                    &trim_roll_rad,
                    &trim_pitch_rad))
            {
                log.level_trim_offset_cdeg[0] =
                    BB_ToInt16(
                        trim_roll_rad,
                        5729.578f);

                log.level_trim_offset_cdeg[1] =
                    BB_ToInt16(
                        trim_pitch_rad,
                        5729.578f);

                log.flags |= (1U << 3);
            }

            YawEstimatorDiagnostics_t yaw_diag;

            YawEstimator_CopyDiagnostics(
                &yaw_diag);

            log.mag_field_reference_mG =
                BB_ToUInt16(
                    yaw_diag.mag_field_reference_gauss,
                    1000.0f);

            log.mag_field_ratio_permille =
                BB_ToUInt16(
                    yaw_diag.mag_field_ratio,
                    1000.0f);

            log.mag_reject_reason =
                yaw_diag.mag_reject_reason;

            if (yaw_diag.initialized)
            {
                log.mag_yaw_cdeg =
                    BB_ToInt16(
                        yaw_diag.mag_yaw_rad,
                        5729.578f);

                log.yaw_mag_innovation_cdeg =
                    BB_ToInt16(
                        yaw_diag.mag_innovation_rad,
                        5729.578f);

                log.mag_field_mG =
                    BB_ToUInt16(
                        yaw_diag.mag_field_norm_gauss,
                        1000.0f);

                log.flags |= (1U << 5);
            }

            if (yaw_diag.mag_accepted)
                log.flags |= (1U << 4);

            log.rate_target_ddps[0] =
                BB_ToInt16(
                    fc.roll_rate_target,
                    10.0f);

            log.rate_target_ddps[1] =
                BB_ToInt16(
                    fc.pitch_rate_target,
                    10.0f);

            log.rate_target_ddps[2] =
                BB_ToInt16(
                    fc.yaw_rate_target,
                    10.0f);

            log.rate_meas_ddps[0] =
                BB_ToInt16(
                    active_data.gx,
                    10.0f);

            log.rate_meas_ddps[1] =
                BB_ToInt16(
                    active_data.gy,
                    10.0f);

            log.rate_meas_ddps[2] =
                BB_ToInt16(
                    active_data.gz,
                    10.0f);

            PID_t *pid[3] = {
                &rate_controller.roll,
                &rate_controller.pitch,
                &rate_controller.yaw};

            for (uint8_t i = 0U; i < 3U; i++)
            {
                log.p_term_duint[i] =
                    BB_ToInt16(
                        pid[i]->kp *
                            pid[i]->error,
                        10.0f);

                log.i_term_duint[i] =
                    BB_ToInt16(
                        pid[i]->ki *
                            pid[i]->integral,
                        10.0f);

                log.d_term_duint[i] =
                    BB_ToInt16(
                        pid[i]->kd *
                            pid[i]->derivative,
                        10.0f);

                log.output_duint[i] =
                    BB_ToInt16(
                        pid[i]->output,
                        10.0f);
            }

            log.motor[0] = m1;
            log.motor[1] = m2;
            log.motor[2] = m3;
            log.motor[3] = m4;

            log.current_dA =
                BB_ToInt16(
                    g_power_health.current_filtered_a,
                    10.0f);

            log.current_limit_permille =
                g_power_health.current_limit_permille;

            if (fc.airmode_active)
                log.flags |= (1U << 0);

            if (g_power_health.current_limiting)
                log.flags |= (1U << 1);

            if (fc.yaw_mode != 0U)
                log.flags |= (1U << 2);

            log.throttle =
                fc.throttle;

            log.limited_throttle =
                limited_throttle;

            log.control_dt_us =
                (uint16_t)(dt * 1000000.0f +
                           0.5f);

            log.active_imu =
                g_imu_health.active_imu_sel;

            log.fresh_imu_flags =
                fresh_imu_flags;

            BB_LogControl(&log);

            /*
             * Navigation Log 在 Control Log 基础上继续分频，
             * 以较低频率记录 GPS、Estimator 和水平控制状态。
             */
            static uint8_t s_bb_nav_log_divider = 0U;

            s_bb_nav_log_divider++;

            if (s_bb_nav_log_divider >=
                BB_NAV_LOG_DIVIDER)
            {
                s_bb_nav_log_divider = 0U;

                BB_NavigationData_t nav_log = {0};

                nav_log.timestamp_cycle =
                    active_data.timestamp_cycle;

                nav_log.rmc_sequence =
                    s_nav_last.rmc_sequence;

                nav_log.gps_velocity_n_cms =
                    BB_ToInt16(
                        s_nav_last.gps_velocity_n_mps,
                        100.0f);

                nav_log.gps_velocity_e_cms =
                    BB_ToInt16(
                        s_nav_last.gps_velocity_e_mps,
                        100.0f);

                nav_log.rmc_velocity_n_cms =
                    BB_ToInt16(
                        s_nav_last.rmc_velocity_n_mps,
                        100.0f);

                nav_log.rmc_velocity_e_cms =
                    BB_ToInt16(
                        s_nav_last.rmc_velocity_e_mps,
                        100.0f);

                nav_log.gps_position_velocity_n_cms =
                    BB_ToInt16(
                        s_nav_last.gps_position_velocity_n_mps,
                        100.0f);

                nav_log.gps_position_velocity_e_cms =
                    BB_ToInt16(
                        s_nav_last.gps_position_velocity_e_mps,
                        100.0f);

                nav_log.gps_velocity_source =
                    s_nav_last.gps_velocity_source;

                nav_log.est_velocity_n_cms =
                    BB_ToInt16(
                        horizontal_estimator.velocity_n_mps,
                        100.0f);

                nav_log.est_velocity_e_cms =
                    BB_ToInt16(
                        horizontal_estimator.velocity_e_mps,
                        100.0f);

                nav_log.accel_n_cms2 =
                    BB_ToInt16(
                        horizontal_estimator.accel_n_mps2,
                        100.0f);

                nav_log.accel_e_cms2 =
                    BB_ToInt16(
                        horizontal_estimator.accel_e_mps2,
                        100.0f);

                nav_log.accel_bias_n_cms2 =
                    BB_ToInt16(
                        horizontal_estimator.accel_bias_n_mps2,
                        100.0f);

                nav_log.accel_bias_e_cms2 =
                    BB_ToInt16(
                        horizontal_estimator.accel_bias_e_mps2,
                        100.0f);

                nav_log.velocity_target_n_cms =
                    BB_ToInt16(
                        s_velocity_target_n_mps,
                        100.0f);

                nav_log.velocity_target_e_cms =
                    BB_ToInt16(
                        s_velocity_target_e_mps,
                        100.0f);

                nav_log.nav_roll_target_cdeg =
                    BB_ToInt16(
                        velocity_controller.roll_target_deg,
                        100.0f);

                nav_log.nav_pitch_target_cdeg =
                    BB_ToInt16(
                        velocity_controller.pitch_target_deg,
                        100.0f);

                nav_log.yaw_cdeg =
                    BB_ToInt16(
                        fc.yaw_meas +
                            NAV_MAG_DECLINATION_DEG,
                        100.0f);

                nav_log.controller_i_n_cms2 =
                    BB_ToInt16(
                        velocity_controller.integral_accel_n_mps2,
                        100.0f);

                nav_log.controller_i_e_cms2 =
                    BB_ToInt16(
                        velocity_controller.integral_accel_e_mps2,
                        100.0f);

                nav_log.gps_age_ms =
                    s_nav_last.rmc_age_ms;

                nav_log.rmc_period_ms =
                    s_nav_last.rmc_period_ms;

                nav_log.gps_hdop_centi =
                    BB_ToUInt16(
                        s_nav_last.gps_hdop,
                        100.0f);

                nav_log.gps_satellites =
                    s_nav_last.gps_satellites;

                nav_log.gps_position_n_cm =
                    BB_ToInt16(
                        horizontal_estimator.gps_position_n_m,
                        100.0f);

                nav_log.gps_position_e_cm =
                    BB_ToInt16(
                        horizontal_estimator.gps_position_e_m,
                        100.0f);

                if (horizontal_estimator.last_position_accepted)
                    nav_log.position_flags |= (1U << 5);

                if (horizontal_estimator.last_position_rejected)
                    nav_log.position_flags |= (1U << 6);

                if (horizontal_estimator
                        .last_position_velocity_correction_applied)
                {
                    nav_log.position_flags |= (1U << 7);
                }

                if (s_nav_last.gps_velocity_valid)
                    nav_log.flags |= (1U << 0);

                if (horizontal_estimator.initialized)
                    nav_log.flags |= (1U << 1);

                if (HorizontalEstimator_IsHealthy(
                        HAL_GetTick()))
                {
                    nav_log.flags |= (1U << 2);
                }

                if (s_velocity_hold_requested)
                    nav_log.flags |= (1U << 3);

                if (s_velocity_hold_active)
                    nav_log.flags |= (1U << 4);

                if (horizontal_estimator.last_gps_accepted)
                    nav_log.flags |= (1U << 5);

#if HORIZONTAL_ESTIMATOR_IMU_PREDICTION_ENABLED

                nav_log.flags |= (1U << 6);

#endif

                if (s_nav_last.gps_velocity_control_ready)
                    nav_log.flags |= (1U << 7);

                nav_log.position_n_cm =
                    BB_ToInt16(
                        horizontal_estimator.position_n_m,
                        100.0f);

                nav_log.position_e_cm =
                    BB_ToInt16(
                        horizontal_estimator.position_e_m,
                        100.0f);

                nav_log.position_target_n_cm =
                    BB_ToInt16(
                        position_controller.target_n_m,
                        100.0f);

                nav_log.position_target_e_cm =
                    BB_ToInt16(
                        position_controller.target_e_m,
                        100.0f);

                nav_log.position_error_n_cm =
                    BB_ToInt16(
                        position_controller.error_n_m,
                        100.0f);

                nav_log.position_error_e_cm =
                    BB_ToInt16(
                        position_controller.error_e_m,
                        100.0f);

                if (s_nav_last.gps_position_control_ready)
                    nav_log.position_flags |= (1U << 0);

                if (s_position_hold_requested)
                    nav_log.position_flags |= (1U << 1);

                if (s_position_hold_active)
                    nav_log.position_flags |= (1U << 2);

                if (position_controller.initialized)
                    nav_log.position_flags |= (1U << 3);

#if POSITION_HOLD_CONTROL_ENABLE

                nav_log.position_flags |= (1U << 4);

#endif

                nav_log.horizontal_mode =
                    (uint8_t)s_horizontal_mode_active;

                nav_log.position_control_phase =
                    (uint8_t)position_controller.phase;

                (void)BB_LogNavigation(&nav_log);
            }
        }
    }
}
