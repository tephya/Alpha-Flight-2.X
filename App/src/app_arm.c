#include "app_arm.h"
#include "app_level_trim.h"
#include "app_rc_calibration.h"
#include "app_imu_calibration.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include <math.h>
#include "cmsis_os2.h"


extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

#define DEBUG_SKIP_ARM_READY_CHECK 1 // TODO：装机带奖试飞前须删除这个宏和Arm_Update中的#if


#define ARM_RC_CHANNEL_SWITCH 4
#define ARM_RC_CHANNEL_THROTTLE 2
#define ARM_SWITCH_ON_THRESHOLD 1000
#define ARM_THROTTLE_LOW_MAX 180
#define ARM_TILT_LIMIT_DEG 30.0f

#define CRASH_CONFIRM_TILT_DEG 50.0f    // 超过50°持续100ms认为飞行器已经失控或倾倒
#define CRASH_HARD_TILT_DEG 65.0f       // 超过65°认为是极端姿态，立即停桨
#define CRASH_CONFIRM_TIME_S 0.10f

volatile ArmState_t g_arm_state = ARM_STATE_DISARMED;

static bool s_switch_seen_off = false;
static float s_crash_tilt_time_s = 0.0f;
static bool s_switch_was_on = false;    // 记录上一次采样时开关状态，用于边沿检测：
                                            // 紧急disarm后若开关仍停留在ON档，不允许电平直接重新解锁
                                            // 必须先见到OFF、再见到ON这个跳变才放行

/**
 * @brief   ARM状态机初始化，上电默认Disarmed
 */
void Arm_Init(void)
{
    g_arm_state = ARM_STATE_DISARMED;
    s_switch_was_on = false;
    s_switch_seen_off = false;
    s_crash_tilt_time_s = 0.0f;
}

static bool Arm_SwitchOn(const RCChannelData_t *rc)
{
    return rc->channels[ARM_RC_CHANNEL_SWITCH] >= ARM_SWITCH_ON_THRESHOLD;
}

static bool Arm_ThrottleLow(const RCChannelData_t *rc)
{
    return rc->channels[ARM_RC_CHANNEL_THROTTLE] <= ARM_THROTTLE_LOW_MAX;
}

static bool Arm_TiltOk(float roll_meas, float pitch_meas)
{
    return (fabsf(roll_meas) < ARM_TILT_LIMIT_DEG &&
            (fabsf)(pitch_meas) < ARM_TILT_LIMIT_DEG);
}

/**
 * @brief   检查是否满足自动停桨状态
 * @retval  true : 已判定为极端倾倒或Crash
 *          false : 尚未满足持续时间
 */
static bool Arm_CrashDetected(float roll_meas, float pitch_meas, float dt)
{
    float abs_roll = fabsf(roll_meas);
    float abs_pitch = fabsf(pitch_meas);
    float max_tilt = (abs_roll > abs_pitch) ? abs_roll : abs_pitch;

    if (max_tilt >= CRASH_HARD_TILT_DEG)
    {
        s_crash_tilt_time_s = 0.0f;
        return true;
    }

    if (max_tilt >= CRASH_CONFIRM_TILT_DEG)
    {
        // 只累计合理的有效IMU dt
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
        s_crash_tilt_time_s = 0.0f; // 姿态恢复正常，重置超限时间
    }

    return false;
}

/**
 * @brief   ARM状态机单次迭代，由Task_FlightCtrl每个控制周期调用一次
 * @note    Disarmed->Armed：RC ARM键 + 低油门 + 倾角<30°，三者同时满足才解锁。
 *          Armed->Disarmed：RC DISARM键。
 *          倾角异常/低压这两条紧急disarm分支目前占位未接，先不会触发
 * @param   rc  最新RC通道数据(Task_FlightCtrl里非阻塞取到的s_rc_last)
 * @param   roll_meas   当前roll角度测量值(°)
 * @param   pitch_meas  当前pitch角度测量值(°)
 */
void Arm_Update(const RCChannelData_t *rc, float roll_meas, float pitch_meas, float dt)
{
    if(g_arm_state == ARM_STATE_DISARMED)
    {
        s_crash_tilt_time_s = 0.0f;

        bool switch_on_now = Arm_SwitchOn(rc);
        // 上电或紧急Disarm后，必须先真实观察到一次OFF
        if(!switch_on_now)
            s_switch_seen_off = true;
        
        bool switch_rising_edge = switch_on_now && !s_switch_was_on && s_switch_seen_off;
        s_switch_was_on = switch_on_now;        // 无论后面是否放行解锁，边沿记录都要更新，
                                                    // 否则未就绪期间的真实跳变会被漏记
        // 失联状态禁止解锁
        if(!rc->link_ok)
            return;

        /* 调试阶段不允许未校准输入解锁 */
        if(!RcCalibration_IsReady() || 
            RcCalibration_IsActive() || 
            !ImuCalibration_IsReady() ||
            LevelTrim_IsActive())
        {
            return;
        }

#if !DEBUG_SKIP_ARM_READY_CHECK
        uint32_t ready_flags = osEventFlagsGet(SystemReadyEventGroupHandle);
        if ((ready_flags & SYSREADY_ARM_MASK) != SYSREADY_ARM_MASK)     // 掩码比较
        {
            return;         // 系统未就绪(GPS/MAG/VBAT/RC_rssi/IMU_health任一未达标)，禁止解锁
        }
#endif
        if(switch_rising_edge && Arm_ThrottleLow(rc) && Arm_TiltOk(roll_meas, pitch_meas))
        {
            s_switch_seen_off = false;

            g_arm_state = ARM_STATE_ARMED;
            BB_LogArmChanged(osKernelGetTickCount(), ARM_STATE_ARMED);
            BB_RequestNewFile();    // 每次解锁开一个新日志文件

            IndicatorEvent_t evt = EVT_ARMED;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }
        return;
    }

    /* 试飞阶段ARM开关OFF就是人工Motor Kill
     * 暂时不要求低油门，也不再等待6s
     *
     * TODO: 试飞成功后，增加防抖Disarm */
    if(!Arm_SwitchOn(rc))
    {
        Arm_ForceDisarm();
        return;
    }

    /* ELRS连续70ms没有有效帧后link_ok变false
     * 当前还没有自动降落能力，因此先采用立即停桨，避免保持旧油门飞走 */
    if(!rc->link_ok)
    {
        Arm_ForceDisarm();
        return;
    }

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

/**
 * @brief   立即撤销解锁并锁存Disarmed状态
 * @note    这里只改变状态并发布非阻塞通知，不直接调用BSP_DSHOT_Send
 *          电机输出统一由FlightCtrl末端安全门控制，避免多个模块争用DSHOT
 */
void Arm_ForceDisarm(void)
{
    // 防止同一故障持续期间重复记录和重复投递事件
    if(g_arm_state != ARM_STATE_ARMED)
        return;

    g_arm_state = ARM_STATE_DISARMED;
    s_crash_tilt_time_s = 0.0f;

    BB_LogArmChanged(osKernelGetTickCount(), ARM_STATE_DISARMED);
    BB_RequestClose();

    IndicatorEvent_t evt = EVT_DISARMED;
    osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}
