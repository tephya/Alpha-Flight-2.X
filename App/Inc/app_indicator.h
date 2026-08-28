/**
 * @file    app_indicator.h
 * @brief   蜂鸣器与 LED 状态提示任务接口。
 */

#ifndef __APP_INDICATOR_H
#define __APP_INDICATOR_H

/**
 * @brief   Indicator 后台任务人口。
 * 
 * 消费系统 Indicator Event，并根据事件类型驱动 Buzzer 与 LED
 * 播放对应提示节拍。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_Indicator_Task(void *argument);

#endif
