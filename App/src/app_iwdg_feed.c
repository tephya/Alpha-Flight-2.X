#include "app_iwdg_feed.h"
#include "app_shared_types.h"
#include "iwdg.h"
#include "cmsis_os2.h"
#include <stdbool.h>

// 心跳超时阈值，FC标称周期1.25ms，乘40倍安全系数得到50ms
#define FLIGHTCTRL_HEARTBEAT_TIMEOUT_MS 50u

// 本任务自身轮询周期，需明显小于心跳超时阈值才能及时发现异常
#define IWDG_FEED_POLL_PERIOD_MS 20U

void App_IwdgFeed_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        uint32_t flightctrl_age = now - g_heartbeat.filghtctrl_last_tick;

        bool all_healthy = (flightctrl_age <= FLIGHTCTRL_HEARTBEAT_TIMEOUT_MS);

        //TODO: res Tasks 加入g_heartbeat后，在这里追加对应的age计算和条件，任一超时都不喂狗
        if(all_healthy)
        {
            HAL_IWDG_Refresh(&hiwdg);
        }

        osDelay(IWDG_FEED_POLL_PERIOD_MS);
    }
}
