/**
 * @file    app_iwdg_feed.c
 * @brief   基于 Task Heartbeat 的 IWDG 条件喂狗实现。
 */

#include "app_iwdg_feed.h"
#include "app_shared_types.h"
#include "iwdg.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/*
 * FlightCtrl 标称周期约 1.25ms。
 * Heartbeat 超时取 50ms，允许一定调度抖动，
 * 同时仍能在控制任务长期卡死时及时停止喂狗。
 */
#define FLIGHTCTRL_HEARTBEAT_TIMEOUT_MS 50U

#define IWDG_FEED_POLL_PERIOD_MS 20U    // IWDG 健康状态轮询周期，ms。

void App_IwdgFeed_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        /*
         * 使用无符号减法计算 Heartbeat Age，
         * 即使 RTOS Tick 发生 uint32_t 回绕也能保持正确的时间差语义。
         */
        const uint32_t flightctrl_age = now - g_heartbeat.flightctrl_last_tick;

        /*
         * 只有所有受监控关键任务均健康时才允许刷新 IWDG。
         * 任一任务 Heartbeat 超时后停止喂狗，
         * 最终由硬件 Watchdog 触发系统复位。
         */
        bool all_healthy = (flightctrl_age <= FLIGHTCTRL_HEARTBEAT_TIMEOUT_MS);

        /*
         * TODO：
         * 其他关键 Task 接入 g_heartbeat后，
         * 在此追加对应 Age 判断，并同一合入 all_healthy。
         */
        
        if(all_healthy)
        {
            HAL_IWDG_Refresh(&hiwdg);
        }

        osDelay(IWDG_FEED_POLL_PERIOD_MS);
    }
}
