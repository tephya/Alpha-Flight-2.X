#include "app_flightctrl.h"
#include "app_imu2_redundancy.h"
#include "app_imu_calibration.h"
#include "app_rc_link.h"
#include "app_arm.h"
#include "app_shared_types.h"
#include "bsp_elrs.h"
#include "bsp_dshot.h"
#include "bsp_qmc5883.h"
#include "bsp_icm42688.h"
#include "bsp_blackbox.h"
#include "alg_attitude.h"
#include "alg_controller.h"
#include "alg_pid.h"
#include "alg_mixer.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <math.h>

/* 测试阶段Airmode使用迟滞：
 * 油门达到30时启用，降到20以下时退出。
 * 避免油门在单一阈值附近抖动造成反复启停。 */
#define AIRMODE_ACTIVATION_THROTTLE 30U
#define AIRMODE_DEACTIVATION_THROTTLE 20U

#include "usart.h"
#include <stdio.h>

extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t RCChannelMailboxHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;

static PID_t pid_yaw;

static FlightControl_t fc = {0};
static RCChannelData_t s_rc_last = {0};     // 非阻塞读取RCChannelMailbox的本地缓存

void FlightControl_Init(void)
{
    AngleController_Init();
    RateController_Init();
    PID_Init(&pid_yaw, 0.0f, 0.0f, 0.0f, 200.0f);

    fc.yaw_mode = 0;
    fc.yaw_target = 0.0f;
    fc.airmode_active = 0U;
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
}

/**
 * @brief   只重建PID微分项的采样基准
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
    if(g_arm_state != ARM_STATE_ARMED)
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
    if(!fc.airmode_active)
    {
        if(fc.throttle < AIRMODE_ACTIVATION_THROTTLE)
        {
            FlightControl_Reset();
            return;
        }

        fc.airmode_active = 1U;
    }
    else if(fc.throttle < AIRMODE_DEACTIVATION_THROTTLE)
    {
        fc.airmode_active = 0U;
        FlightControl_Reset();
        return;
    }

    // 默认自动偏航(遥控SA键未按下)
    if(s_rc_last.channels[5]>1500)
    {
        // 手动偏航
        fc.yaw_mode = 1;
        fc.yaw_rate_target = Map_Yaw(s_rc_last.channels[3]);
        if(fabsf(fc.yaw_rate_target) < 5.0f)
            fc.yaw_rate_target = 0.0f;
    }
    else
    {
        /* 自动偏航；
         * 第一次进入Heading Hold，锁定当前航向*/
        if(fc.yaw_mode)
        {
            fc.yaw_target = fc.yaw_meas;
            pid_yaw.integral = 0.0f;
        }
        fc.yaw_mode = 0;

        fc.yaw_rate_target = YawHeadingHold_Update(&pid_yaw, fc.yaw_target, fc.yaw_meas, dt);
    }

    /* 角度环(PID外环) */
    AngleController_Update(fc.roll_target, fc.pitch_target,
                           fc.roll_meas, fc.pitch_meas, dt);

    fc.roll_rate_target = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /* 角速度环(内环) */
    RateController_Update(fc.roll_rate_target, fc.pitch_rate_target, fc.yaw_rate_target,
                          gx, gy, gz, dt);

    fc.roll_cmd = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd = rate_controller.yaw_output;
}

static int16_t BB_ToInt16(float value, float scale)
{
    float scaled = value * scale;

    if(scaled > 32767.0f)
        return 32767;
    if(scaled < -32768.0f)
        return -32768;

    return (int16_t)scaled;
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

        if(ImuCalibration_IsReady())
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);
        else
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_GYRO_CALIB_OK);

        if(imu_result == IMU_UPDATE_CALIBRATION)
        {
            /* 校准期间禁止任何控制输出；移动机体只会延长等待时间。 */
            FlightControl_Reset();
            BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            continue;
        }

        if(imu_result == IMU_UPDATE_TIMING_ANOMALY)
        {
            PID_ResetDerivativeOnly(&angle_controller.roll);
            PID_ResetDerivativeOnly(&angle_controller.pitch);

            PID_ResetDerivativeOnly(&rate_controller.roll);
            PID_ResetDerivativeOnly(&rate_controller.pitch);
            PID_ResetDerivativeOnly(&rate_controller.yaw);

            PID_ResetDerivativeOnly(&pid_yaw);

            if(timing_anomaly_streak < 0xFFU)
                timing_anomaly_streak++;

            /* 本帧不做姿态积分和PID，保持上一帧电机输出；
             * 连续异常说明控制调度已经不可信 */
            if(timing_anomaly_streak >= 3U)
            {
                Arm_ForceDisarm();
                BSP_DSHOT_Send(0U, 0U, 0U, 0U);
            }

            continue;
        }

        if(imu_result == IMU_UPDATE_STANDBY_ONLY)
        {
            /* 待命-IMU刷新不能再次运行姿态解算和PID
             * 保持上一帧电机输出，等待active IMU的新数据 */
            continue;
        }

        if(imu_result == IMU_UPDATE_ACTIVE_FRAME)
        {
            /* 只有新的active帧且dt正常，才解除连续时序异常计数 */
            timing_anomaly_streak = 0U;
        }

        if((imu_result  == IMU_UPDATE_TIMEOUT) || (imu_result == IMU_UPDATE_DUAL_FAULT))
        {
            if((imu_result == IMU_UPDATE_DUAL_FAULT) || g_imu_health.dual_fault)
            {
                Arm_ForceDisarm();
            }

            if(g_arm_state != ARM_STATE_ARMED)

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

        if(!attitude_initialized)
        {
            /* 首帧以崇礼方向初始化，避免从0度缓慢收敛造成虚假误差。 */
            attitude.roll = attitude.accel_roll;
            attitude.pitch = attitude.accel_pitch;
            attitude_initialized = true;
        }
        else
        {
            Attitude_Update(&active_data, &attitude, dt);
        }

        fc.roll_meas = attitude.roll * 57.29578f;
        fc.pitch_meas = attitude.pitch * 57.29578f;

        // 获取Mag数据
        MagData_t mag;
        if (osMessageQueueGet(MagDataMailboxHandle, &mag, NULL, 0) == osOK)
        {
            Attitude_CptYaw(&mag, &attitude);
            fc.yaw_meas = attitude.yaw * 57.29578f;
        }

        // 获取RC数据
        osMessageQueueGet(RCChannelMailboxHandle, &s_rc_last, NULL, 0);

        Arm_Update(&s_rc_last, fc.roll_meas, fc.pitch_meas, dt);

        /* Arm_Update可能因为人工Motor Kill、RC lost、低压、
         * 双IMU故障或倾倒检测而撤销解锁
         * 
         * 一旦撤销，本周期立即发送DShot 0，
         * 不再进入PID、Mixer和正常电机输出 */
        if(g_arm_state != ARM_STATE_ARMED)
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

        if(current_limit < 600U || current_limit > 1000U)
            current_limit = 1000U;

        /* 只缩放油门，Mixer仍能优先维持姿态控制
        * 使用uint32_t完成乘法，避免uint16_t乘法结果溢出 */
        uint16_t limited_throttle = (uint16_t)(((uint32_t)fc.throttle * current_limit) / 1000U);

        Mixer(fc.roll_cmd, fc.pitch_cmd, fc.yaw_cmd, limited_throttle, fc.airmode_active, &m1, &m2, &m3, &m4);

        BSP_DSHOT_Send((uint16_t)(m1 + 48U), (uint16_t)(m2 + 48U),
                       (uint16_t)(m3 + 48U), (uint16_t)(m4 + 48U));

        /**
         * 控制环标称800Hz，按4分频记录约200Hz。
         * 日志字段全部取自本控制周期，避免跨Task读取fc产生数据撕裂。
         * BB_LogControl只负责packed记录到RAM缓冲，不在FlightCtrl中执行SD I/O。
         */
        #define BB_LOG_DIVIDER 4U
        static uint8_t s_bb_log_divider = 0U;

        s_bb_log_divider++;

        if(s_bb_log_divider >= BB_LOG_DIVIDER)
        {
            s_bb_log_divider = 0U;

            BB_ControlData_t log = {0};

            log.timestamp_cycle = active_data.timestamp_cycle;

            log.angle_cdeg[0] = BB_ToInt16(fc.roll_meas, 100.0f);
            log.angle_cdeg[1] = BB_ToInt16(fc.pitch_meas, 100.0f);
            log.angle_cdeg[2] = BB_ToInt16(fc.yaw_meas, 100.0f);

            log.angle_target_cdeg[0] = BB_ToInt16(fc.roll_target, 100.0f);
            log.angle_target_cdeg[1] = BB_ToInt16(fc.pitch_target, 100.0f);

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

            if(fc.airmode_active)
                log.flags |= (1U << 0);

            if(g_power_health.current_limiting)
                log.flags |= (1U << 1);

            log.throttle = fc.throttle;
            log.limited_throttle = limited_throttle;
            log.control_dt_us = (uint16_t)(dt * 1000000.0f + 0.5f);
            log.active_imu = g_imu_health.active_imu_sel;
            log.fresh_imu_flags = fresh_imu_flags;

            BB_LogControl(&log);
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
