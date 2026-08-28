/**
 * @file    app_iwdg_feed.h
 * @brief   IWDG 喂狗任务接口。
 */

#ifndef __APP_IWDG_FEED_H
#define __APP_IWDG_FEED_H

/**
 * @brief   IWDG 喂狗后台任务入口。
 * 
 * 周期检查关键 Task Heartbeat；
 * 仅当所有受监控任务均处于健康状态时刷新 IWDG。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_IwdgFeed_Task(void *argument);

#endif
