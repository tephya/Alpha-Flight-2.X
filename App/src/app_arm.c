#include "app_arm.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stdbool.h>

#define ARM_RC_CHANNEL_SWITCH 4
#define ARM_RC_CHANNEL_THROTTLE 2
#define ARM_SWITCH_ON_THRESHOLD 1000
#define ARM_THROTTLE_LOW_MAX 180
#define ARM_TILT_LIMIT_DEG 30.0f
#define ARM_DISARM_HOLD_MS 6000U

volatile ArmState_t g_arm_state = ARM_STATE_DISARMED;

static bool s_disarmed_hold_active = false;
static uint32_t s_disarmed_hold_start_tick = 0;

/**
 * @brief   ARM状态机初始化，上电默认Disarmed
 */
void Arm_Init(void)
{
    g_arm_state = ARM_STATE_DISARMED;
    s_disarmed_hold_active = false;
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
 * @brief   ARM状态机单次迭代，由Task_FlightCtrl每个控制周期调用一次
 * @note    Disarmed->Armed：RC ARM键 + 低油门 + 倾角<30°，三者同时满足才解锁。
 *          Armed->Disarmed：RC DISARM键 + 低油门，持续满6秒（正常落地退出）。
 *          倾角异常/低压这两条紧急disarm分支目前占位未接，先不会触发
 * @param   rc  最新RC通道数据(Task_FlightCtrl里非阻塞取到的s_rc_last)
 * @param   roll_meas   当前roll角度测量值(°)
 * @param   pitch_meas  当前pitch角度测量值(°)
 */
void Arm_Update(const RCChannelData_t *rc, float roll_meas, float pitch_meas)
{
    if(g_arm_state == ARM_STATE_DISARMED)
    {
        s_disarmed_hold_active = false;     // Disarmed期间不需要计时

        if(g_imu_health.dual_fault)
        {
            return;         // 双路IMU失效，数据不可信，禁止解锁
        }

        if(Arm_SwitchOn(rc) && Arm_ThrottleLow(rc) && Arm_TiltOk(roll_meas, pitch_meas))
        {
            g_arm_state = ARM_STATE_ARMED;
        }
        return;
    }

    if(g_imu_health.dual_fault)
    {
        g_arm_state = ARM_STATE_DISARMED;
        s_disarmed_hold_active = false;
        return;             // 立即disarm，不走下面的6sdebounce
    }

    /**
     * TODO: 倾角异常/低压紧急disarm占位
     */

    if(!Arm_SwitchOn(rc) && Arm_ThrottleLow(rc))
    {
        if(!s_disarmed_hold_active)
        {
            s_disarmed_hold_active = true;
            s_disarmed_hold_start_tick = osKernelGetTickCount();
        }
        else if((osKernelGetTickCount() - s_disarmed_hold_start_tick) >= ARM_DISARM_HOLD_MS)
        {
            g_arm_state = ARM_STATE_DISARMED;
            s_disarmed_hold_active = false;
        }
    }
    else{
        s_disarmed_hold_active = false;         // 条件中断，计时重来
    }
}
