#include "app_flightctrl.h"
#include "bsp_icm42688.h"
#include "app_imu2_redundancy.h"
#include "bsp_debug_uart.h"

void App_FlightCtrl_Task(void *argument)
{
    (void)argument;

    /* 上电初始化：复位+配置寄存器+建立事件标志对象。
     * fail_mask非0说明某颗IMU的WHO_AM_I校验没过，SPI通信有问题，
     * 测试阶段先不处理这个返回值，实际飞控代码需要在这里加错误处理/指示灯报警 */
    uint8_t fail_mask = ICM_InitAll();
    (void)fail_mask; /* TODO: 测试阶段暂不处理，后续需要在这里对接故障指示 */

    ImuRedundancy_Init();

    IcmData_t active_data;
    float dt;

    for (;;)
    {
        bool ok = ImuRedundancy_Update(&active_data, &dt);

        if (ok)
        {
            /* active_data是当前应使用的姿态数据，测试阶段暂不消费它，
             * 后续接Attitude_CptYaw等解算函数时从这里往下接。
             * 交叉比对打印(DebugUart_PrintImuDiff)是否已经按之前说的
             * 手动加进ImuRedundancy_Update里了，少爷确认一下 */
        }

        /* 测试阶段：循环体到这里结束，下一轮由osEventFlagsWait本身阻塞节流，
         * 不需要额外osDelay */
    }
}
