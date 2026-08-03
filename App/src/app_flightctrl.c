#include "app_flightctrl.h"
#include "app_imu2_redundancy.h"
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

#define AIRMODE_ACTIVATION_THROTTLE 30U

#include "usart.h"
#include <stdio.h>

extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t RCChannelMailboxHandle;
extern osMessageQueueId_t SystemReadyEventGroupHandle;

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
    // 在ARM状态下超过激活阈值，保持Airmode锁定
    if(!fc.airmode_active)
    {
        if(fc.throttle < AIRMODE_ACTIVATION_THROTTLE)
        {
            FlightControl_Reset();
            return;
        }

        fc.airmode_active = 1U;
    }

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
        // 第一次进入Heading Hold，锁定当前航向
        if(fc.yaw_mode)
        {
            fc.yaw_target = fc.yaw_meas;
            pid_yaw.integral = 0.0f;
        }
        fc.yaw_mode = 0;

        fc.yaw_rate_target = YawHeadingHold_Update(&pid_yaw, fc.yaw_target, fc.yaw_meas, dt);
    }

    /**
     * 角度环(PID外环)
     */
    AngleController_Update(fc.roll_target, fc.pitch_target,
                           fc.roll_meas, fc.pitch_meas, dt);

    fc.roll_rate_target = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /**
     * 角速度环(内环)
     */
    RateController_Update(fc.roll_rate_target, fc.pitch_rate_target, fc.yaw_rate_target,
                          gx, gy, gz, dt);

    fc.roll_cmd = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd = rate_controller.yaw_output;
}

void App_FlightCtrl_Task(void *argument)
{
    (void)argument;

    /* 上电初始化：复位+配置寄存器+建立事件标志对象。
     * fail_mask非0说明某颗IMU的WHO_AM_I校验没过，SPI通信有问题，
     * 测试阶段先不处理这个返回值，实际飞控代码需要在这里加错误处理/指示灯报警 */
    uint8_t fail_mask = ICM_InitAll();
    (void)fail_mask;

    ImuRedundancy_Init();
    FlightControl_Init();
    Arm_Init();
    BSP_DSHOT_Init();

    osDelay(5000);
    // ESC上电握手：持续发0油门，让ESC完成自检
    for (int i = 0; i < 3000; i++)
    {
        BSP_DSHOT_Send(0, 0, 0, 0);
        g_heartbeat.flightctrl_last_tick = osKernelGetTickCount();
        osDelay(1);
    }

    IcmData_t active_data;
    float dt;
    uint16_t m1, m2, m3, m4;

    for (;;)
    {
        bool ok = ImuRedundancy_Update(&active_data, &dt);
        g_heartbeat.flightctrl_last_tick = osKernelGetTickCount();

        // 检测IMU_OK_BIT
        if (g_imu_health.dual_fault)
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);
        else
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_IMU_HEALTH_OK);

        /* IMU偶发单次timeout时先跳过本轮控制；
         * 达到dual_fault判定后立即撤销解锁并持续发送DShot0 */
        if (!ok)
        {
            if(g_imu_health.dual_fault)
                Arm_ForceDisarm();

            if(g_arm_state != ARM_STATE_ARMED)
                BSP_DSHOT_Send(0U, 0U, 0U, 0U);

            continue;
        }

        Attitude_ComputeAccelAngles(&active_data, &attitude);
        Attitude_Update(&active_data, &attitude, dt);

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

        Mixer(fc.roll_cmd, fc.pitch_cmd, fc.yaw_cmd, limited_throttle, &m1, &m2, &m3, &m4);

        BSP_DSHOT_Send((uint16_t)(m1 + 48U), (uint16_t)(m2 + 48U),
                       (uint16_t)(m3 + 48U), (uint16_t)(m4 + 48U));

        uint16_t time_ms = (uint16_t)osKernelGetTickCount();
        int16_t angle_cdeg[3] = {
            (int16_t)(fc.roll_meas * 100.0f),
            (int16_t)(fc.pitch_meas * 100.0f),
            (int16_t)(fc.yaw_meas * 100.0f)};

        uint16_t motor[4] = {m1, m2, m3, m4};
        int8_t target_cdeg[3] = {
            (int8_t)fc.roll_target,
            (int8_t)fc.pitch_target,
            (int8_t)fc.yaw_rate_target};

        BB_LogMotion(time_ms, angle_cdeg, motor, target_cdeg);

        /* 测试阶段：循环体到这里结束，下一轮由osEventFlagsWait本身阻塞节流，
            * 不需要额外osDelay */
        }
}

void FlightControl_CopyTo(FlightControl_t *out)
{
    *out = fc;
    // TODO: 由于fc结构体较大，所以后续可以增加TYPE参数，根据需要拷贝
}
