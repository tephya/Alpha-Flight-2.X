/**
 * @file    app_power_monitor.h
 * @brief   电源监测，低压保护及电流软限流任务接口。
 */

#ifndef __APP_POWER_MONITOR_H
#define __APP_POWER_MONITOR_H

/**
 * @brief   Power Monitor 后台任务入口。
 * 
 * 周期采集电池电压与整机电流，维护 Voltage Health，
 * Low Battery Warning，Current Limiter 及对应 Indicator Event。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_PowerMonit_Task(void *argument);

#endif
