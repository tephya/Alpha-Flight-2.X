/**
 * @file    app_rc_link.h
 * @brief   RC Link 后台任务接口。
 */

#ifndef __APP_RC_LINK_H
#define __APP_RC_LINK_H

/**
 * @brief   RC link 后台任务入口。
 * 
 * 周期消费 ELRS 接收数据，推进 RC Calibration，
 * 发布最新归一化 RC 状态，并维护 RC Link / Calibration Ready 状态。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_RcLink_Task(void *argument);

#endif
