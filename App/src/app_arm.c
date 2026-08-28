/**
 * @file    app_arm.c
 * @brief   飞行器 ARM 状态机及运行时停桨保护实现。
 */

#include "app_arm.h"
#include "app_level_trim.h"
#include "app_rc_calibration.h"
#include "app_imu_calibration.h"
#include "app_mag_calibration.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include <math.h>

extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

/* RC 解锁输入及基础安全门限。 */
#define ARM_RC_CHANNEL_SWITCH 4         // ARM 开关通道索引。
#define ARM_RC_CHANNEL_THROTTLE 2       // Throttle 通道索引。
#define ARM_SWITCH_ON_THRESHOLD 1000    // ARM 开关判定为 ON 的通道阈值。
#define ARM_THROTTLE_LOW_MAX 200        // 允许解锁的最大低油门通道值。
#define ARM_TILT_LIMIT_DEG 30.0f        // 允许解锁的最大 Roll/Pitch 绝对倾角，deg。

/*
 * Crash Protection：
 * 中等异常倾角需持续一定时间确认，极端倾角则立即触发停桨。
 */
#define CRASH_CONFIRM_TILT_DEG 50.0f    // 进入持续 Crash 确认的倾角阈值，deg。
#define CRASH_HARD_TILT_DEG 65.0f       // 立即判定 Crash 的极端倾角阈值，deg。
#define CRASH_CONFIRM_TIME_S 0.10f      // 异常倾角持续确认时间，s。

/* 当前全局 ARM 状态。 */
volatile ArmState_t g_arm_state = ARM_STATE_DISARMED;

/*
 * 解锁开关使用 OFF -> ON 边沿触发，而非单纯 ON 水平。
 * 上电或紧急 Disarm 后必须先真实观察到一次 OFF，
 * 防止 ARM 开关仍停留在 ON 时自动重新解锁。
 */
static bool s_switch_seen_off = false;
static bool s_switch_was_on = false;

/* Crash 倾角连续超限累计时间，s。 */
static float s_crash_tilt_time_s = 0.0f;

void Arm_Init(void)
{
    g_arm_state = ARM_STATE_DISARMED;
    s_switch_was_on = false;
    s_switch_seen_off = false;
    s_crash_tilt_time_s = 0.0f;
}

/* 判断当前 ARM 开关是否处于 ON。 */
static bool Arm_SwitchOn(const RCChannelData_t *rc)
{
    return rc->channels[ARM_RC_CHANNEL_SWITCH] >= ARM_SWITCH_ON_THRESHOLD;
}

/* 判断当前油门是否满足安全解锁条件。 */
static bool Arm_ThrottleLow(const RCChannelData_t *rc)
{
    return rc->channels[ARM_RC_CHANNEL_THROTTLE] <= ARM_THROTTLE_LOW_MAX;
}

/* 判断当前 Roll/Pitch 是否处于允许解锁的姿态范围。 */
static bool Arm_TiltOk(float roll_meas, float pitch_meas)
{
    return (fabsf(roll_meas) < ARM_TILT_LIMIT_DEG &&
            (fabsf)(pitch_meas) < ARM_TILT_LIMIT_DEG);
}

/* 
 * 检测飞行器是否进入需要立即停桨的异常倾角状态。
 * 
 * 超过 Hard Tilt 时立即判定 Crash；
 * 超过 Confirm Tilt 但未达到 Hard Tilt 时，需持续超限达到确认时间。
 */
static bool Arm_CrashDetected(float roll_meas, float pitch_meas, float dt)
{
    float abs_roll = fabsf(roll_meas);
    float abs_pitch = fabsf(pitch_meas);
    float max_tilt = (abs_roll > abs_pitch) ? abs_roll : abs_pitch;

    // 极端姿态无需等待持续时间，立即触发停桨。
    if (max_tilt >= CRASH_HARD_TILT_DEG)
    {
        s_crash_tilt_time_s = 0.0f;
        return true;
    }

    if (max_tilt >= CRASH_CONFIRM_TILT_DEG)
    {
        // 仅累计合理控制周期，避免异常 Dt 直接触发 Crash。
        if (dt > 0.0f && dt <= 0.02f)
            s_crash_tilt_time_s += dt;

        if (s_crash_tilt_time_s >= CRASH_CONFIRM_TIME_S)
        {
            s_crash_tilt_time_s = 0.0f;
            return true;
        }
    }
    else
    {
        // 倾角恢复到确定门限以内后，连续超限计时重新开始。
        s_crash_tilt_time_s = 0.0f;
    }

    return false;
}

void Arm_Update(const RCChannelData_t *rc, float roll_meas, float pitch_meas, float dt)
{
    if(g_arm_state == ARM_STATE_DISARMED)
    {
        s_crash_tilt_time_s = 0.0f;

        const bool switch_on_now = Arm_SwitchOn(rc);

        // 上电或紧急 Disarm 后，必须先真实观察到一次 ARM OFF。
        if(!switch_on_now)
            s_switch_seen_off = true;
        
        /*
         * 只有观察到有效 OFF -> ON 跳变才产生解锁请求。
         * 单纯保持 ON 电平不能触发重新解锁。
         */
        const bool switch_rising_edge = 
            switch_on_now && 
            !s_switch_was_on && 
            s_switch_seen_off;

        /*
         * 无论后续安全条件是否允许解锁，都必须更新边沿历史，
         * 否则系统未就绪期间发生的真实开关变化会被错误保留。
         */
        s_switch_was_on = switch_on_now;

        // RC 链路失效时禁止进入 Armed。
        if(!rc->link_ok)
            return;

        /*
         * 校准过程未完成或仍处于活动状态时禁止解锁，
         * 防止使用未经确认的传感器/遥控输入进入闭环控制。
         */
        if(!RcCalibration_IsReady() || 
            RcCalibration_IsActive() || 
            !ImuCalibration_IsReady() ||
            LevelTrim_IsActive() ||
            MagCalibration_IsActive())
        {
            return;
        }

        const uint32_t ready_flags = osEventFlagsGet(SystemReadyEventGroupHandle);

        /*
         * 所有 SYSREADY_ARM_MASK 条件必须同时满足。
         * 任一关键系统状态未就绪时禁止解锁。
         */
        if ((ready_flags & SYSREADY_ARM_MASK) != SYSREADY_ARM_MASK)
        {
            return;
        }

        /*
         * 最终解锁条件：
         * 有效 OFF -> ON 边沿 + 低油门 + 安全倾角。
         */
        if(switch_rising_edge && 
            Arm_ThrottleLow(rc) && 
            Arm_TiltOk(roll_meas, pitch_meas))
        {
            // 本次边沿已消费，下一次解锁前必须重新观察到 OFF。
            s_switch_seen_off = false;

            g_arm_state = ARM_STATE_ARMED;

            BB_LogArmChanged(osKernelGetTickCount(), ARM_STATE_ARMED);

            // 每次新的 Armed 周期使用独立 Blackbox 文件。
            BB_RequestNewFile();

            IndicatorEvent_t evt = EVT_ARMED;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }
        return;
    }

    /*
     * 当前将 ARM 开关 OFF 直接作为人工 Motor Kill。
     * 暂不要求低油门或额外持续确认。
     * 
     * 后续可考虑加入 Disarm 防抖。
     */
    if(!Arm_SwitchOn(rc))
    {
        Arm_ForceDisarm();
        return;
    }

    /*
     * ELRS 连续约 70ms 无有效帧后 link_ok 失效。
     * 当前没有可靠自动降落能力，因此失联后立即停桨，
     * 避免持续使用最后一帧 RC 指令。
     */
    if(!rc->link_ok)
    {
        Arm_ForceDisarm();
        return;
    }

    // 双 IMU 同时故障或电源电压故障均视为不可继续飞行。
    if(g_imu_health.dual_fault || g_power_health.voltage_fault)
    {
        Arm_ForceDisarm();
        return;
    }

    if(Arm_CrashDetected(roll_meas, pitch_meas, dt))
    {
        Arm_ForceDisarm();
        return;
    }
}

void Arm_ForceDisarm(void)
{
    // 避免同一故障持续期间重复记录日志和重复发布事件。
    if(g_arm_state != ARM_STATE_ARMED)
        return;

    g_arm_state = ARM_STATE_DISARMED;
    s_crash_tilt_time_s = 0.0f;

    BB_LogArmChanged(osKernelGetTickCount(), ARM_STATE_DISARMED);
    BB_RequestClose();

    IndicatorEvent_t evt = EVT_DISARMED;
    osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}
