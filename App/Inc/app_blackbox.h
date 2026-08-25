/**
 * @file   app_blackbox.h
 * @brief   Blackbox 后台任务及文件状态查询接口。
 */

#ifndef __APP_BLACKBOX_H
#define __APP_BLACKBOX_H

#include <stdbool.h>

/**
 * @brief   查询当前 Blackbox 日志文件是否已打开。
 *
 * @return  true 表示当前存在打开的日志文件。
 */
bool App_Blackbox_IsFileOpen(void);

/**
 * @brief   查询最近一次日志文件关闭操作是否成功。
 *
 * @return  true 表示最近一次 BB_Close() 成功完成。
 */
bool App_Blackbox_LastCloseSucceeded(void);

/**
 * @brief   Blackbox 后台任务入口。
 * 
 * 负责相应新建/关闭日志文件请求，并将 READY 缓冲区持续写入 SD Card。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_Blackbox_Task(void *argument);

#endif
