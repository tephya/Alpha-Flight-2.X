/**
 * @file    bsp_debug_uart.c
 * @brief   双 IMU 差值调试输出实现。
 */

#include "bsp_debug_uart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/**
 * Debug UART 打印分频。
 * 
 * 若调用频率为 800 Hz，则每 8 次调用输出一次，
 * 对应约 100 Hz 的调试数据输出频率。
 */
#define DEBUG_PRINT_DIVIDER 8

void DebugUart_PrintImuDiff(const IcmData_t *d1, const IcmData_t *d2)
{
    if(d1 == NULL || d2 == NULL)
    {
        return;
    }

    static uint32_t s_counter = 0;

    s_counter++;

    /*
     * 仅每 DEBUG_PRINT_DIVIDER 次调用执行一次格式化与 UART 发送，
     * 降低高频控制路径中的调试输出开销。
     */
    if (s_counter % DEBUG_PRINT_DIVIDER != 0)
    {
        return;
    }

    char buf[256];

    /*
     * 输出两路 IMU 对应轴的直接差距：
     * 
     * d_* = IMU1 - IMU2
     * 
     * 这些数据主要用于离线观察正常状态下的双 IMU 分布范围，
     * 从而辅助确定 CrossCheck Threshold。
     */
    const int len = snprintf(buf, sizeof(buf),
                   "d_gx=%.3f d_gy=%.3f d_gz=%.3f d_ax=%.4f d_ay=%.4f d_az=%.4f\r\n",
                   (double)(d1->gx - d2->gx),
                   (double)(d1->gy - d2->gy),
                   (double)(d1->gz - d2->gz),
                   (double)(d1->ax - d2->ax),
                   (double)(d1->ay - d2->ay),
                   (double)(d1->az - d2->az));

    /*
     * snprintf() 返回“本来需要写入的字符数”。
     * 只有结果大于 0 且确实完整落入 Buffer 时才发送，
     * 避免发截断后仍按过大的 len 读取 Buffer。
     */
    if (len > 0 &&
        len < (int)sizeof(buf))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 10);
    }
}
