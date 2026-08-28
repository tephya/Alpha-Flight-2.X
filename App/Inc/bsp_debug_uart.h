/**
 * @file    bsp_debug_uart.h
 * @brief   IMU 调试 UART 输出接口。
 */

#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H

#include "bsp_icm42688.h"

/**
 * @brief   输出双 IMU 六轴惯量测量差值。
 * 
 * 用于测试阶段观察 IMU1 / IMU2 的 Gyro 与 Accel 差异，
 * 辅助确定 IMU Redundancy CrossCheck 阈值。
 * 
 * @param[in] d1    IMU1 最新测量数据。
 * @param[in] d2    IMU2 最新测量数据。
 * 
 * @note    函数内部带有打印分频，并非每次调用都会执行 UART 发送。
 */
void DebugUart_PrintImuDiff(const IcmData_t *d1, const IcmData_t *d2);

#endif
