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

/* 测试阶段Airmode使用迟滞：
 * 油门达到30时启用，降到20以下时退出。
 * 避免油门在单一阈值附近抖动造成反复启停。 */
#define AIRMODE_ACTIVATION_THROTTLE 30U

#define AIRMODE_DEACTIVATION_THROTTLE 20U

#define FLIGHT_ACCEL_BIAS_SETTLE_TIME_S 1.0f

/* 第一阶段必须保持0：Estimator和日志运行，但不覆盖Roll/Pitch目标。
 * 无桨数据确认GPS更新率、N/E方向和Accel符号后才改1。 */
#define VELOCITY_HOLD_CONTROL_ENABLE 1U
#define POSITION_HOLD_CONTROL_ENABLE 1U

#define HORIZONTAL_MODE_RC_CHANNEL 8U // SC: CRSF CH9 -> channels[8]
#define HORIZONTAL_MODE_SC_MIDDLE_MIN 700U
#define HORIZONTAL_MODE_SC_MIDDLE_MAX 1300U
#define HORIZONTAL_MODE_SC_HIGH_MIN 1600U
#define HORIZONTAL_PILOT_ACCEL_LIMIT_MPS2 6.0f

#define HORIZONTAL_MODE_ENTRY_MAX_VELOCITY_ERROR_MPS 0.50f
#define HORIZONTAL_MODE_ENTRY_MAX_POSITION_ERROR_M 1.00f
#define HORIZONTAL_MODE_ENTRY_GPS_ACCEPT_STREAK 3U
#define HORIZONTAL_MODE_ENTRY_POSITION_ACCEPT_STREAK 3U
#define HORIZONTAL_MODE_BLEND_TIME_S 0.50f

#define POSITION_BRAKE_GPS_VELOCITY_DISAGREEMENT_MAX_MPS 0.30f
#define HORIZONTAL_BIAS_STATIONARY_TIME_S 1.0f
#define HORIZONTAL_BIAS_GYRO_MAX_DPS 3.0f
#define HORIZONTAL_BIAS_ACCEL_NORM_MIN_G 0.85f
#define HORIZONTAL_BIAS_ACCEL_NORM_MAX_G 1.15f
#define HORIZONTAL_BIAS_GPS_SPEED_MAX_MPS 0.15f

#define VELOCITY_HOLD_MAX_SPEED_MPS 2.0f
#define POSITION_HOLD_PILOT_SPEED_MPS 1.50f
#define HORIZONTAL_MODE_ENTRY_MIN_THROTTLE 400U

#define HORIZONTAL_PILOT_INPUT_MAX_DEG 30.0f
#define HORIZONTAL_PILOT_DEADZONE_DEG 2.0f
#define HORIZONTAL_PILOT_EXPO 0.50f

/* RMC course是True North基准，当前Mag Yaw是Magnetic North基准。
 * 东偏为正：True heading = Magnetic heading + declination
 * 第一阶段观测暂用0；正式接管前必须填写测试地点对应值。 */
#define NAV_MAG_DECLINATION_DEG 0.0f

/* CONTROL日志约为200Hz，Navigation记录按10分频约20Hz。 */
#define BB_NAV_LOG_DIVIDER 10U

#define NAV_SHADOW_FORCE_DSHOT_ZERO 0U

#include "usart.h"
#include <stdio.h>

extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t RCChannelMailboxHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t NavStateMailboxHandle;

typedef enum
{
    HORIZONTAL_MODE_MANUAL = 0,
    HORIZONTAL_MODE_VELOCITY_HOLD = 1,
    HORIZONTAL_MODE_POSITION_HOLD = 2,
} FlightHorizontalMode_t;

static PID_t pid_yaw;

static FlightControl_t fc = {0};
static RCChannelData_t s_rc_last = {0}; // 非阻塞读取RCChannelMailbox的本地缓存

static NavState_t s_nav_last = {0};
static uint8_t s_velocity_hold_requested;
static uint8_t s_velocity_hold_active;
static uint8_t s_position_hold_requested;
static uint8_t s_position_hold_active;

static FlightHorizontalMode_t s_horizontal_mode_active = HORIZONTAL_MODE_MANUAL;
static FlightHorizontalMode_t s_horizontal_mode_last_request = HORIZONTAL_MODE_MANUAL;
static uint8_t s_horizontal_manual_seen;

static float s_velocity_target_n_mps;
static float s_velocity_target_e_mps;
static float s_pilot_velocity_cmd_n_mps;
static float s_pilot_velocity_cmd_e_mps;

static float s_flight_accel_bias_hold_time_s;

static float s_disarmed_accel_bias_stationary_time_s;
static float s_horizontal_control_blend;

void FlightControl_Init(void)
{
    AngleController_Init();
    RateController_Init();
    HorizontalEstimator_Init();
    VelocityController_Init();
    PositionController_Init();
    /* 先初始化Mag Cali，在把参考场强传给YawEstimator */
    MagCalibration_Init();
    YawEstimator_Init(MagCalibration_GetFieldReferenceGauss());
    /* 最大输出限制为30°/s，避免航向误差直接要求过大的Yaw Rate */
    PID_Init(&pid_yaw, 1.0f, 0.0f, 0.0f, 30.0f);

    fc.yaw_mode = 0;
    fc.yaw_target = 0.0f;
    fc.airmode_active = 0U;

    s_disarmed_accel_bias_stationary_time_s = 0.0f;
    s_horizontal_control_blend = 0.0f;
}

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
    /* 只复位Velocity Controller，不复位Estimator；
     * 否则Disarmed步行测试无法观察速度估计。 */
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
}

/**
 * @brief   只重建PID微分项的采样基准
 *
 * @note    用于控制周期异常后的恢复。保留Integral和上一帧Output，
 *          避免一次调度抖动导致整个控制器状态被清空。
 *          first_update使下一次PID_Update先对齐last_measurement，
 *          从而避免跨越异常时间间隔计算出Derivative尖峰。
 */
static void PID_ResetDerivativeOnly(PID_t *pid)
{
    pid->derivative = 0.0f;
    pid->derivative_raw = 0.0f;
    pid->derivative_lpf = 0.0f;
    pid->first_update = 1U;
}

static void FlightControl_UpdateHorizontalEstimator(const IcmData_t *imu, float dt)
{
    NavState_t nav;
    if (osMessageQueueGet(NavStateMailboxHandle, &nav, NULL, 0U) == osOK)
        s_nav_last = nav;

    if (!YawEstimator_IsInitialized())
    {
        HorizontalEstimator_Reset();
        return;
    }

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

    const bool imu_stationary =
        fabsf(imu->gx) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        fabsf(imu->gy) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        fabsf(imu->gz) <= HORIZONTAL_BIAS_GYRO_MAX_DPS &&
        accel_norm_sq >= accel_norm_min_sq &&
        accel_norm_sq <= accel_norm_max_sq;

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

    const bool flight_accel_bias_context_valid =
        (g_arm_state == ARM_STATE_ARMED) &&
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) &&
        (position_controller.phase == POSITION_CONTROL_PHASE_HOLD) &&
        (s_nav_last.gps_velocity_valid != 0U) &&
        (s_nav_last.gps_velocity_control_ready != 0U) &&
        (s_nav_last.gps_position_velocity_valid != 0U) &&
        (gps_velocity_candidate_disagreement_mps <= 
            POSITION_BRAKE_GPS_VELOCITY_DISAGREEMENT_MAX_MPS);

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

    const bool allow_estimator_state_reacquire =
        (s_horizontal_mode_active == HORIZONTAL_MODE_MANUAL);

    if (s_nav_last.rmc_sequence != horizontal_estimator.last_gps_sequence)
    {
        (void)HorizontalEstimator_CorrectGps(
            s_nav_last.gps_velocity_n_mps,
            s_nav_last.gps_velocity_e_mps,
            s_nav_last.rmc_sequence,
            s_nav_last.rmc_last_update_ms,
            navigation_yaw_rad,
            s_nav_last.gps_velocity_valid != 0U,
            allow_flight_accel_bias_correction,
            allow_estimator_state_reacquire);
    }

    if (s_nav_last.gps_position_control_ready == 0U)
    {
        /* Position质量失效后清除局部参考系;恢复时从当前坐标重新建立,
         * 避免拿长时间失锁前的局部状态吸收一次巨大innovation. */
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

static void FlightControl_RefreshHorizontalModeFlags(FlightHorizontalMode_t requested)
{
    s_velocity_hold_requested =
        (requested == HORIZONTAL_MODE_VELOCITY_HOLD) ? 1U : 0U;
    s_position_hold_requested =
        (requested == HORIZONTAL_MODE_POSITION_HOLD) ? 1U : 0U;

    s_velocity_hold_active =
        (s_horizontal_mode_active == HORIZONTAL_MODE_VELOCITY_HOLD) ? 1U : 0U;
    s_position_hold_active =
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) ? 1U : 0U;
}

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

    const bool rmc_course_usable =
        rmc_speed_mps >= NAV_RMC_VECTOR_MIN_SPEED_MPS;
    const float observed_velocity_n_mps =
        rmc_course_usable ? s_nav_last.gps_velocity_n_mps : 0.0f;
    const float observed_velocity_e_mps =
        rmc_course_usable ? s_nav_last.gps_velocity_e_mps : 0.0f;

    const float error_n_mps =
        observed_velocity_n_mps - horizontal_estimator.velocity_n_mps;
    const float error_e_mps =
        observed_velocity_e_mps - horizontal_estimator.velocity_e_mps;
    const float error_mps = sqrtf(
        error_n_mps * error_n_mps +
        error_e_mps * error_e_mps);

    return error_mps >= 0.0f &&
           error_mps <=
               HORIZONTAL_MODE_ENTRY_MAX_VELOCITY_ERROR_MPS;
}

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

/**
 * @brief   尝试进入指定水平辅助模式
 *
 * @note    从Manual进入时清空Velocity Controller，避免带入旧控制状态；
 *          Velocity Hold与Position Hold直接切换时保留Velocity Integral，
 *          实现平滑切换并保留已建立的静态抗风补偿。
 */
static bool FlightControl_TryEnterHorizontalMode(FlightHorizontalMode_t requested, bool estimator_healthy)
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
         * 从Manual进入时必须清除旧Velocity状态。
         * 从Position切换过来时保留Integral及上一帧输出，
         * 防止已经建立的抗风补偿突然消失。
         */
        if (entering_from_manual)
            VelocityController_Reset();

        PositionController_Reset();
        s_horizontal_mode_active = HORIZONTAL_MODE_VELOCITY_HOLD;
        return true;
    }

    if (requested == HORIZONTAL_MODE_POSITION_HOLD)
    {
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
         * Manual进入Position时从干净状态开始；
         * Velocity转Position时保留内层速度环的抗风补偿、
         */
        if (entering_from_manual)
            VelocityController_Reset();
        else
            VelocityController_ResetIntegral();

        s_horizontal_mode_active = HORIZONTAL_MODE_POSITION_HOLD;
        return true;
    }

    return false;
}

/**
 * @brief   对水平摇杆应用Deadzone和expo，改善Position Hold精细修正手感。
 * @note    expo只压低中杆附近灵敏度；满杆仍保持原速度上限。
 */
static float FlightControl_ApplyHorizontalPliotDeadzone(float input_deg)
{
    const float magnitude = fabsf(input_deg);

    if (magnitude <= HORIZONTAL_PILOT_DEADZONE_DEG)
        return 0.0f;

    float normalized =
        (magnitude - HORIZONTAL_PILOT_DEADZONE_DEG) /
        (HORIZONTAL_PILOT_INPUT_MAX_DEG - HORIZONTAL_PILOT_DEADZONE_DEG);
    normalized = fminf(normalized, 1.0f);

    const float curved =
        (1.0f - HORIZONTAL_PILOT_EXPO) * normalized +
        HORIZONTAL_PILOT_EXPO * normalized * normalized * normalized;
    const float scaled = curved * HORIZONTAL_PILOT_INPUT_MAX_DEG;

    return (input_deg < 0.0f) ? -scaled : scaled;
}

static void FlightControl_GetPilotVelocityNe(float max_speed_mps,
                                             float yaw_rad,
                                             float *velocity_n_mps,
                                             float *velocity_e_mps)
{
    const float pitch_input_deg =
        FlightControl_ApplyHorizontalPliotDeadzone(
            Map_Pitch(s_rc_last.channels[1]));
    const float roll_input_deg =
        FlightControl_ApplyHorizontalPliotDeadzone(
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

/**
 * @brief   限制Pilot Velocity command的二维变化率。
 *
 * @note    这里只整形飞手输入，不限制Position/Velocity Controller的反馈
 *          Acceleration，因此风扰反馈仍可立即使用完整控制权限。
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

static void FlightControl_UpdateHorizontalMode(float dt)
{
    const FlightHorizontalMode_t requested =
        FlightControl_GetRequestedHorizontalMode();

    const bool request_changed =
        requested != s_horizontal_mode_last_request;
    s_horizontal_mode_last_request = requested;

    const bool estimator_healthy =
        HorizontalEstimator_IsHealthy(HAL_GetTick());

    // 只有明确观察到SC低档后，才允许下一次辅助模式切入
    if (requested == HORIZONTAL_MODE_MANUAL)
    {
        s_horizontal_manual_seen = 1U;

        if (s_horizontal_mode_active != HORIZONTAL_MODE_MANUAL)
            FlightControl_ExitHorizontalMode();

        FlightControl_RefreshHorizontalModeFlags(requested);
        return;
    }

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
            // GPS、Estimator或油门条件失效：
            // 退出后必须先回SC低档，禁止自动恢复接管。
            FlightControl_ExitHorizontalMode();
            s_horizontal_manual_seen = 0U;
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }

        if (requested != s_horizontal_mode_active)
        {
            /*
             * SC从低档拨向高档时可能短暂经过中档。
             * 允许Velocity和Position直接切换，避免中档先接管后
             * 又把高档判定为非法跨档
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
        /* 从Manual进入辅助模式仍要求：
         * 先明确观察到SC低档，再出现新的辅助档切换沿。 */
        if (!request_changed || !s_horizontal_manual_seen)
        {
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }

        // 一次切入沿只允许尝试一次；失败后也必须回低档重试
        s_horizontal_manual_seen = 0U;

        if (!FlightControl_TryEnterHorizontalMode(requested, estimator_healthy))
        {
            FlightControl_RefreshHorizontalModeFlags(requested);
            return;
        }
    }

    const float yaw_rad =
        (fc.yaw_meas + NAV_MAG_DECLINATION_DEG) * 0.0174532925f;

    float pilot_velocity_target_n_mps;
    float pilot_velocity_target_e_mps;

    const float pilot_speed_limit_mps =
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) ? POSITION_HOLD_PILOT_SPEED_MPS : VELOCITY_HOLD_MAX_SPEED_MPS;

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

    if((s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) &&
        ((position_controller.phase == POSITION_CONTROL_PHASE_MOVING) ||
        (position_controller.phase == POSITION_CONTROL_PHASE_BRAKING)))
    {
        VelocityController_DecayIntegral(dt);
    }

    const bool allow_velocity_integral_learning =
        (s_horizontal_mode_active == HORIZONTAL_MODE_VELOCITY_HOLD) ||
        ((s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) &&
         (position_controller.integral_learning_allowed != 0U));

    VelocityController_Update(
        s_velocity_target_n_mps,
        s_velocity_target_e_mps,
        horizontal_estimator.velocity_n_mps,
        horizontal_estimator.velocity_e_mps,
        horizontal_estimator.accel_n_mps2,
        horizontal_estimator.accel_e_mps2,
        yaw_rad,
        allow_velocity_integral_learning,
        dt);

    FlightControl_RefreshHorizontalModeFlags(requested);

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

/**
 * @brief   根据当前RC、姿态和Gyro数据运行串级控制器
 * @note    调用顺序固定为RC映射、Airmode状态机、Yaw模式、
 *          Angle外环、Rate内环。本函数只计算控制量，不直接发送DShot。
 */
static void FlightControl_Update(float dt, float gx, float gy, float gz)
{
    fc.roll_target = Map_Roll(s_rc_last.channels[0]);
    fc.pitch_target = Map_Pitch(s_rc_last.channels[1]);
    fc.throttle = Map_Throttle(s_rc_last.channels[2]);

    // ARM不要启动姿态控制电机
    if (g_arm_state != ARM_STATE_ARMED)
    {
        fc.airmode_active = 0U;
        FlightControl_Reset();
        return;
    }

    /* 测试阶段的Airmode状态机。
     *
     * 未启用时，油门达到激活阈值才开始运行姿态PID。
     * 已启用后，油门降到退出阈值以下便停止PID并清空控制状态。
     *
     * 注意：这是近地调试策略。正式飞行时，空中低油门仍需姿态控制，
     * 后续应由airborne/landed状态决定是否退出Airmode。*/
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

    // 默认自动偏航(遥控SA键未按下)
    if (s_rc_last.channels[5] > 1500)
    {
        // 手动偏航
        fc.yaw_mode = 1U;
        fc.yaw_rate_target = Map_Yaw(s_rc_last.channels[3]);
        if (fabsf(fc.yaw_rate_target) < 5.0f)
            fc.yaw_rate_target = 0.0f;
    }
    else
    {
        // 自动偏航；
        if (fc.yaw_mode != 0U)
        {
            /* 手动Yaw退出时以当前航向为新锁定点，并清除两级Yaw控制器的
             * 历史状态，避免模式切换把旧积分或微分带入 */
            fc.yaw_target = fc.yaw_meas;
            PID_Reset(&pid_yaw);
            rate_controller.yaw.integral = 0.0f;
            PID_ResetDerivativeOnly(&rate_controller.yaw);
        }

        fc.yaw_mode = 0;
        fc.yaw_rate_target = YawHeadingHold_Update(
            &pid_yaw, fc.yaw_target, fc.yaw_meas, dt);
    }

    /* 角度环(PID外环) */
    AngleController_Update(fc.roll_target, fc.pitch_target,
                           fc.roll_meas, fc.pitch_meas, dt);

    fc.roll_rate_target = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /* 当前仅在Position Hold中使用Rate Feedforward，
     * 保持Manual和Velocity Hold原有控制手感不变。 */
    RateController_SetRollPitchFeedForwardEnabled(
        (s_horizontal_mode_active == HORIZONTAL_MODE_POSITION_HOLD) ? 1U : 0U);

    /* 角速度环(内环) */
    RateController_Update(fc.roll_rate_target,
                          fc.pitch_rate_target,
                          fc.yaw_rate_target,
                          gx, gy, gz, dt);

    fc.roll_cmd = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd = rate_controller.yaw_output;
}

static int16_t BB_ToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
        return 32767;
    if (scaled < -32768.0f)
        return -32768;

    return (int16_t)scaled;
}

static uint16_t BB_ToUInt16(float value, float scale)
{
    const float scaled = value * scale;

    if (scaled <= 0.0f)
        return 0U;
    if (scaled >= 65535.0f)
        return 65535U;

    return (uint16_t)(scaled + 0.5f);
}

/**
 * @brief   FlightCtrl主任务
 * @note    循环由双IMU DRDY EventFlags驱动，不使用osDelay节流。
 *          只有active IMU产生新时间戳时才运行姿态解算和PID；
 *          standby刷新、超时、双故障及时间异常均在进入控制算法前处理。
 */
void App_FlightCtrl_Task(void *argument)
{
    (void)argument;

    /* 上电初始化：复位+配置寄存器+建立事件标志对象。
     * fail_mask非0说明某颗IMU的WHO_AM_I校验没过，SPI通信有问题，
     * 测试阶段先不处理这个返回值，实际飞控代码需要在这里加错误处理/指示灯报警 */
    uint8_t fail_mask = ICM_InitAll();

    ImuRedundancy_Init(fail_mask);
    FlightControl_Init();
    Arm_Init();

    static uint8_t timing_anomaly_streak = 0U; // IMU读取超时计数
    IcmData_t active_data;
    float dt;
    uint16_t m1, m2, m3, m4;
    bool attitude_initialized = false;

    for (;;)
    {
        uint8_t fresh_imu_flags = 0U;

        /**
         * WaitAny会被任意一颗IMU唤醒。返回结果不仅表示通信是否成功，
         * 还负责区分active新帧、standby单独刷新和active时间异常，
         * 防止同一份active缓存被重复送入姿态解算和PID。
         */
        ImuUpdateResult_t imu_result = ImuRedundancy_Update(&active_data, &dt, &fresh_imu_flags);
        g_heartbeat.flightctrl_last_tick = osKernelGetTickCount();

        // 检测IMU_OK_BIT
        if (g_imu_health.dual_fault)
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);
        else
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);

        if (ImuCalibration_IsReady())
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);
        else
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);

        if (imu_result == IMU_UPDATE_CALIBRATION)
        {
            /* 校准期间禁止任何控制输出；移动机体只会延长等待时间。 */
            FlightControl_Reset();
            BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            continue;
        }

        if (imu_result == IMU_UPDATE_TIMING_ANOMALY)
        {
            PID_ResetDerivativeOnly(&angle_controller.roll);
            PID_ResetDerivativeOnly(&angle_controller.pitch);

            PID_ResetDerivativeOnly(&rate_controller.roll);
            PID_ResetDerivativeOnly(&rate_controller.pitch);
            PID_ResetDerivativeOnly(&rate_controller.yaw);

            PID_ResetDerivativeOnly(&pid_yaw);

            if (timing_anomaly_streak < 0xFFU)
                timing_anomaly_streak++;

            /* 本帧不做姿态积分和PID，保持上一帧电机输出；
             * 连续异常说明控制调度已经不可信 */
            if (timing_anomaly_streak >= 3U)
            {
                Arm_ForceDisarm();
                BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            }

            continue;
        }

        if (imu_result == IMU_UPDATE_STANDBY_ONLY)
        {
            /* 待命-IMU刷新不能再次运行姿态解算和PID
             * 保持上一帧电机输出，等待active IMU的新数据 */
            continue;
        }

        if (imu_result == IMU_UPDATE_ACTIVE_FRAME)
        {
            /* 只有新的active帧且dt正常，才解除连续时序异常计数 */
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

        /**
         * 到达这里必然是时间有效的active新帧。
         * dt来自active IMU连续两次实际读取时间戳，而不是Task唤醒间隔。
         */
        Attitude_ComputeAccelAngles(&active_data, &attitude);

        /* Trim只能作用于Accel角，不能直接修改最终姿态角或Gyro数据 */
        LevelTrim_Apply((IcmInstance_t)g_imu_health.active_imu_sel,
                        &attitude.accel_roll,
                        &attitude.accel_pitch);

        if (!attitude_initialized)
        {
            /* 首帧以崇礼方向初始化，避免从0度缓慢收敛造成虚假误差。 */
            attitude.roll = attitude.accel_roll;
            attitude.pitch = attitude.accel_pitch;
            attitude_initialized = true;
        }
        else
        {
            /*
             * Armed飞行期间减弱Accel修正，防止水平运动加速度污染Roll/Pitch;
             * Disarmed时恢复正常修正，使姿态重新收敛到重力方向。
             */
            Attitude_Update(
                &active_data,
                &attitude,
                dt,
                g_arm_state == ARM_STATE_ARMED);
        }

        fc.roll_meas = attitude.roll * 57.29578f;
        fc.pitch_meas = attitude.pitch * 57.29578f;

        // 获取Mag数据
        // 每个有效active IMU帧都用Gyro推进Yaw；Mag只在50Hz新样本到达时慢校正。
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

            if (g_arm_state == ARM_STATE_DISARMED)
            {
                fc.yaw_target = fc.yaw_meas;
            }
        }

        FlightControl_UpdateHorizontalEstimator(&active_data, dt);

        // 获取RC数据
        osMessageQueueGet(RCChannelMailboxHandle, &s_rc_last, NULL, 0);
        MagCalibration_HandleRc(&s_rc_last);
        LevelTrim_HandleRc(&s_rc_last);

        Arm_Update(&s_rc_last, fc.roll_meas, fc.pitch_meas, dt);

        /* Arm_Update可能因为人工Motor Kill、RC lost、低压、
         * 双IMU故障或倾倒检测而撤销解锁
         *
         * 一旦撤销，本周期立即发送DShot 0，
         * 不再进入PID、Mixer和正常电机输出 */
        if (g_arm_state != ARM_STATE_ARMED)
        {
            FlightControl_Reset();
            BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            continue;
        }

        FlightControl_Update(dt, active_data.gx, active_data.gy, active_data.gz);

        /**
         * 获取PowerMonitor发布的油门比例。
         * 正常范围为600~1000；越界表示数据尚未初始化或被破坏，
         * 此时回退到100%，避免错误地把飞行油门清零。
         */
        uint16_t current_limit = g_power_health.current_limit_permille;

        if (current_limit < 600U || current_limit > 1000U)
            current_limit = 1000U;

        /* 只缩放油门，Mixer仍能优先维持姿态控制
         * 使用uint32_t完成乘法，避免uint16_t乘法结果溢出 */
        uint16_t limited_throttle = (uint16_t)(((uint32_t)fc.throttle * current_limit) / 1000U);

        Mixer(fc.roll_cmd, fc.pitch_cmd, fc.yaw_cmd, limited_throttle, fc.airmode_active, &m1, &m2, &m3, &m4);

#if NAV_SHADOW_FORCE_DSHOT_ZERO
        BSP_DSHOT_Send(0U, 0U, 0U, 0U);
#else
        BSP_DSHOT_Send((uint16_t)(m1 + 48U), (uint16_t)(m2 + 48U),
                       (uint16_t)(m3 + 48U), (uint16_t)(m4 + 48U));
#endif
/**
 * 控制环标称800Hz，按4分频记录约200Hz。
 * 日志字段全部取自本控制周期，避免跨Task读取fc产生数据撕裂。
 * BB_LogControl只负责packed记录到RAM缓冲，不在FlightCtrl中执行SD I/O。
 */
#define BB_LOG_DIVIDER 4U
        static uint8_t s_bb_log_divider = 0U;

        s_bb_log_divider++;

        if (s_bb_log_divider >= BB_LOG_DIVIDER)
        {
            s_bb_log_divider = 0U;

            BB_ControlData_t log = {0};

            log.timestamp_cycle = active_data.timestamp_cycle;

            log.angle_cdeg[0] = BB_ToInt16(fc.roll_meas, 100.0f);
            log.angle_cdeg[1] = BB_ToInt16(fc.pitch_meas, 100.0f);
            log.angle_cdeg[2] = BB_ToInt16(fc.yaw_meas, 100.0f);

            log.angle_target_cdeg[0] = BB_ToInt16(fc.roll_target, 100.0f);
            log.angle_target_cdeg[1] = BB_ToInt16(fc.pitch_target, 100.0f);
            log.angle_target_cdeg[2] = BB_ToInt16(fc.yaw_target, 100.0f);

            float trim_roll_rad;
            float trim_pitch_rad;

            if (LevelTrim_GetOffsets(
                    (IcmInstance_t)g_imu_health.active_imu_sel,
                    &trim_roll_rad,
                    &trim_pitch_rad))
            {
                log.level_trim_offset_cdeg[0] =
                    BB_ToInt16(trim_roll_rad, 5729.578f);
                log.level_trim_offset_cdeg[1] =
                    BB_ToInt16(trim_pitch_rad, 5729.578f);
                log.flags |= (1U << 3);
            }

            YawEstimatorDiagnostics_t yaw_diag;
            YawEstimator_CopyDiagnostics(&yaw_diag);

            log.mag_field_reference_mG = BB_ToUInt16(yaw_diag.mag_field_reference_gauss, 1000.0f);
            log.mag_field_ratio_permille = BB_ToUInt16(yaw_diag.mag_field_ratio, 1000.0f);
            log.mag_reject_reason = yaw_diag.mag_reject_reason;

            if (yaw_diag.initialized)
            {
                log.mag_yaw_cdeg = BB_ToInt16(
                    yaw_diag.mag_yaw_rad, 5729.578f);
                log.yaw_mag_innovation_cdeg = BB_ToInt16(
                    yaw_diag.mag_innovation_rad, 5729.578f);
                log.mag_field_mG = BB_ToUInt16(
                    yaw_diag.mag_field_norm_gauss, 1000.0f);
                log.flags |= (1U << 5);
            }

            if (yaw_diag.mag_accepted)
                log.flags |= (1U << 4);

            log.rate_target_ddps[0] = BB_ToInt16(fc.roll_rate_target, 10.0f);
            log.rate_target_ddps[1] = BB_ToInt16(fc.pitch_rate_target, 10.0f);
            log.rate_target_ddps[2] = BB_ToInt16(fc.yaw_rate_target, 10.0f);

            log.rate_meas_ddps[0] = BB_ToInt16(active_data.gx, 10.0f);
            log.rate_meas_ddps[1] = BB_ToInt16(active_data.gy, 10.0f);
            log.rate_meas_ddps[2] = BB_ToInt16(active_data.gz, 10.0f);

            PID_t *pid[3] = {
                &rate_controller.roll,
                &rate_controller.pitch,
                &rate_controller.yaw};

            for (uint8_t i = 0U; i < 3U; i++)
            {
                log.p_term_duint[i] = BB_ToInt16(pid[i]->kp * pid[i]->error, 10.0f);
                log.i_term_duint[i] = BB_ToInt16(pid[i]->ki * pid[i]->integral, 10.0f);
                log.d_term_duint[i] = BB_ToInt16(pid[i]->kd * pid[i]->derivative, 10.0f);
                log.output_duint[i] = BB_ToInt16(pid[i]->output, 10.0f);
            }

            log.motor[0] = m1;
            log.motor[1] = m2;
            log.motor[2] = m3;
            log.motor[3] = m4;

            log.current_dA = BB_ToInt16(g_power_health.current_filtered_a, 10.0f);
            log.current_limit_permille = g_power_health.current_limit_permille;

            if (fc.airmode_active)
                log.flags |= (1U << 0);

            if (g_power_health.current_limiting)
                log.flags |= (1U << 1);

            if (fc.yaw_mode != 0U)
                log.flags |= (1U << 2);

            log.throttle = fc.throttle;
            log.limited_throttle = limited_throttle;
            log.control_dt_us = (uint16_t)(dt * 1000000.0f + 0.5f);
            log.active_imu = g_imu_health.active_imu_sel;
            log.fresh_imu_flags = fresh_imu_flags;

            BB_LogControl(&log);

            static uint8_t s_bb_nav_log_divider = 0U;
            s_bb_nav_log_divider++;

            if (s_bb_nav_log_divider >= BB_NAV_LOG_DIVIDER)
            {
                s_bb_nav_log_divider = 0U;

                BB_NavigationData_t nav_log = {0};
                nav_log.timestamp_cycle = active_data.timestamp_cycle;
                nav_log.rmc_sequence = s_nav_last.rmc_sequence;

                nav_log.gps_velocity_n_cms =
                    BB_ToInt16(s_nav_last.gps_velocity_n_mps, 100.0f);
                nav_log.gps_velocity_e_cms =
                    BB_ToInt16(s_nav_last.gps_velocity_e_mps, 100.0f);
                nav_log.rmc_velocity_n_cms =
                    BB_ToInt16(s_nav_last.rmc_velocity_n_mps, 100.0f);
                nav_log.rmc_velocity_e_cms =
                    BB_ToInt16(s_nav_last.rmc_velocity_e_mps, 100.0f);
                nav_log.gps_position_velocity_n_cms =
                    BB_ToInt16(s_nav_last.gps_position_velocity_n_mps, 100.0f);
                nav_log.gps_position_velocity_e_cms =
                    BB_ToInt16(s_nav_last.gps_position_velocity_e_mps, 100.0f);
                nav_log.gps_velocity_source = s_nav_last.gps_velocity_source;
                nav_log.est_velocity_n_cms =
                    BB_ToInt16(horizontal_estimator.velocity_n_mps, 100.0f);
                nav_log.est_velocity_e_cms =
                    BB_ToInt16(horizontal_estimator.velocity_e_mps, 100.0f);
                nav_log.accel_n_cms2 =
                    BB_ToInt16(horizontal_estimator.accel_n_mps2, 100.0f);
                nav_log.accel_e_cms2 =
                    BB_ToInt16(horizontal_estimator.accel_e_mps2, 100.0f);
                nav_log.accel_bias_n_cms2 =
                    BB_ToInt16(horizontal_estimator.accel_bias_n_mps2, 100.0f);
                nav_log.accel_bias_e_cms2 =
                    BB_ToInt16(horizontal_estimator.accel_bias_e_mps2, 100.0f);

                nav_log.velocity_target_n_cms =
                    BB_ToInt16(s_velocity_target_n_mps, 100.0f);
                nav_log.velocity_target_e_cms =
                    BB_ToInt16(s_velocity_target_e_mps, 100.0f);
                nav_log.nav_roll_target_cdeg =
                    BB_ToInt16(velocity_controller.roll_target_deg, 100.0f);
                nav_log.nav_pitch_target_cdeg =
                    BB_ToInt16(velocity_controller.pitch_target_deg, 100.0f);
                nav_log.yaw_cdeg = BB_ToInt16(
                    fc.yaw_meas + NAV_MAG_DECLINATION_DEG, 100.0f);
                nav_log.controller_i_n_cms2 = BB_ToInt16(
                    velocity_controller.integral_accel_n_mps2, 100.0f);
                nav_log.controller_i_e_cms2 = BB_ToInt16(
                    velocity_controller.integral_accel_e_mps2, 100.0f);

                nav_log.gps_age_ms = s_nav_last.rmc_age_ms;
                nav_log.rmc_period_ms = s_nav_last.rmc_period_ms;
                nav_log.gps_hdop_centi = BB_ToUInt16(s_nav_last.gps_hdop, 100.0f);
                nav_log.gps_satellites = s_nav_last.gps_satellites;
                nav_log.gps_position_n_cm = BB_ToInt16(
                    horizontal_estimator.gps_position_n_m, 100.0f);
                nav_log.gps_position_e_cm = BB_ToInt16(
                    horizontal_estimator.gps_position_e_m, 100.0f);

                if (horizontal_estimator.last_position_accepted)
                    nav_log.position_flags |= (1U << 5);
                if (horizontal_estimator.last_position_rejected)
                    nav_log.position_flags |= (1U << 6);
                if (horizontal_estimator.last_position_velocity_correction_applied)
                    nav_log.position_flags |= (1U << 7);
                if (s_nav_last.gps_velocity_valid)
                    nav_log.flags |= (1U << 0);
                if (horizontal_estimator.initialized)
                    nav_log.flags |= (1U << 1);
                if (HorizontalEstimator_IsHealthy(HAL_GetTick()))
                    nav_log.flags |= (1U << 2);
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

                nav_log.position_n_cm = BB_ToInt16(
                    horizontal_estimator.position_n_m, 100.0f);
                nav_log.position_e_cm = BB_ToInt16(
                    horizontal_estimator.position_e_m, 100.0f);
                nav_log.position_target_n_cm = BB_ToInt16(
                    position_controller.target_n_m, 100.0f);
                nav_log.position_target_e_cm = BB_ToInt16(
                    position_controller.target_e_m, 100.0f);
                nav_log.position_error_n_cm = BB_ToInt16(
                    position_controller.error_n_m, 100.0f);
                nav_log.position_error_e_cm = BB_ToInt16(
                    position_controller.error_e_m, 100.0f);

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

        /* 测试阶段：循环体到这里结束，下一轮由osEventFlagsWait本身阻塞节流，
         * 不需要额外osDelay */
    }
}

void FlightControl_CopyTo(FlightControl_t *out)
{
    *out = fc;
    // TODO: 由于fc结构体较大，所以后续可以增加TYPE参数，根据需要拷贝
}
