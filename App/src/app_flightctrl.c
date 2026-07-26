#include "app_flightctrl.h"
#include "app_imu2_redundancy.h"
#include "app_rc_link.h"
#include "app_arm.h"
#include "app_shared_types.h"
#include "bsp_elrs.h"
#include "bsp_dshot.h"
#include "bsp_qmc5883.h"
#include "bsp_icm42688.h"
#include "alg_attitude.h"
#include "alg_controller.h"
#include "alg_pid.h"
#include "alg_mixer.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <math.h>

extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t RCChannelMailboxHandle;

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
}

void FlightControl_Reset(void)
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
    fc.throttle = 0;

    fc.yaw_target = fc.yaw_meas;
    fc.yaw_mode = 0;

    PID_Reset(&pid_yaw);
}

static void FlightControl_Update(float dt, float gx, float gy, float gz)
{
    fc.roll_target = Map_Roll(s_rc_last.channels[0]);
    fc.pitch_target = Map_Pitch(s_rc_last.channels[1]);
    fc.throttle = Map_Throttle(s_rc_last.channels[2]);

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
     * 角度环(PID外环)。pitch轴按旧板贴装规律取负，roll轴不变；
     */
    AngleController_Update(fc.roll_target, fc.pitch_target,
                           fc.roll_meas, -fc.pitch_meas, dt);

    fc.roll_rate_target = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /**
     * 角速度环(内环)。gy同理取负，gx/gz不变
     */
    RateController_Update(fc.roll_rate_target, fc.pitch_rate_target, fc.yaw_rate_target,
                          gx, -gy, gz, dt);

    fc.roll_cmd = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd = rate_controller.yaw_output;

    /**
     * 动态限幅，油门太低时持续复位控制器，防止地面怠速时积分饱和；
     * 油门起来后，姿态修正量的允许幅度随油门余量动态放宽(上限250)
     */
    if(fc.throttle < 30)
    {
        FlightControl_Reset();
    }
    else
    {
        float dynamic_limit = (float)(fc.throttle - 30);
        if(dynamic_limit > 250.0f)
            dynamic_limit = 250.0f;
        
        if(fc.roll_cmd > dynamic_limit)
            fc.roll_cmd = dynamic_limit;
        if (fc.roll_cmd < -dynamic_limit)
            fc.roll_cmd = -dynamic_limit;

        if (fc.pitch_cmd > dynamic_limit)
            fc.pitch_cmd = dynamic_limit;
        if (fc.pitch_cmd < -dynamic_limit)
            fc.pitch_cmd = -dynamic_limit;

        if (fc.yaw_cmd > dynamic_limit)
            fc.yaw_cmd = dynamic_limit;
        if (fc.yaw_cmd < -dynamic_limit)
            fc.yaw_cmd = -dynamic_limit;
    }
}

void App_FlightCtrl_Task(void *argument)
{
    (void)argument;

    /* 上电初始化：复位+配置寄存器+建立事件标志对象。
     * fail_mask非0说明某颗IMU的WHO_AM_I校验没过，SPI通信有问题，
     * 测试阶段先不处理这个返回值，实际飞控代码需要在这里加错误处理/指示灯报警 */
    uint8_t fail_mask = ICM_InitAll();
    (void)fail_mask; /* TODO: 测试阶段暂不处理，后续需要在这里对接故障指示 */

    ImuRedundancy_Init();
    FlightControl_Init();
    Arm_Init();
    BSP_DSHOT_Init();

    IcmData_t active_data;
    float dt;
    uint16_t m1, m2, m3, m4;

    for (;;)
    {
        bool ok = ImuRedundancy_Update(&active_data, &dt);
        g_heartbeat.flightctrl_last_tick = osKernelGetTickCount();
        if (ok)
        {
            Attitude_ComputeAccelAngles(&active_data, &attitude);
            Attitude_Update(&active_data, &attitude, dt);

            fc.roll_meas = attitude.roll * 57.29578f;
            fc.pitch_meas = attitude.pitch * 57.29578f;
            
            // 获取Mag数据
            MagData_t mag;
            if(osMessageQueueGet(MagDataMailboxHandle, &mag, NULL, 0) == osOK)
            {
                Attitude_CptYaw(&mag, &attitude);
                fc.yaw_meas = attitude.yaw * 57.29578f;
            }

            // 获取RC数据
            osMessageQueueGet(RCChannelMailboxHandle, &s_rc_last, NULL, 0);

            Arm_Update(&s_rc_last, fc.roll_meas, fc.pitch_meas);

            FlightControl_Update(dt, active_data.gx, active_data.gy, active_data.gz);

            Mixer(fc.roll_cmd, fc.pitch_cmd, fc.yaw_cmd, fc.throttle, &m1, &m2, &m3, &m4);

            if(g_arm_state == ARM_STATE_ARMED)
            {
                BSP_DSHOT_Send((uint16_t)(m1 + 48U), (uint16_t)(m2 + 48U),
                               (uint16_t)(m3 + 48U), (uint16_t)(m4 + 48U));
            }
            else
            {
                BSP_DSHOT_Send(0, 0, 0, 0);
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
