/**
 * @file    app_nav.h
 * @brief   Navigation 后台任务接口。
 */

#ifndef __APP_NAV_H
#define __APP_NAV_H

/**
 * @brief   Navigation 后台任务入口。
 * 
 * 周期处理 GPS NMEA，Home 状态，GPS Velocity Observation，
 * QMC Mag 数据读取与发布，并维护相关 System Ready 状态。
 * 
 * @param[in] arguemnt  RTOS Task 参数，当前未使用。
 */
void App_Nav_Task(void *argument);

#endif
