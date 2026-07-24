#include "bsp_debug_uart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/* 分频：每N帧打印一次。800Hz/8=100Hz刷新，测试阶段够用；
 * 想改密度只改这一个数，不用动其他逻辑 */
#define DEBUG_PRINT_DIVIDER 8

/**
 * @brief   纯测试用：打印IMU1/IMU2六轴数据及差值，供推算交叉验证阈值。
 * @note    内部自带分频，不是每次调用都真的发送。
 *          调试完毕、阈值定下来后，这个模块和对它的调用整体删除，不进正式版本。 
 * @param   d1 姿态数据结构体1
 * @param   d2 姿态数据结构体2
 */
void DebugUart_PrintImuDiff(const IcmData_t *d1, const IcmData_t *d2)
{
    static uint32_t s_counter = 0;
    char buf[256];
    int len;

    s_counter++;
    if (s_counter % DEBUG_PRINT_DIVIDER != 0)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "d_gx=%.3f d_gy=%.3f d_gz=%.3f d_ax=%.4f d_ay=%.4f d_az=%.4f\r\n",
                   (double)(d1->gx - d2->gx),
                   (double)(d1->gy - d2->gy),
                   (double)(d1->gz - d2->gz),
                   (double)(d1->ax - d2->ax),
                   (double)(d1->ay - d2->ay),
                   (double)(d1->az - d2->az));

    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 10);
    }
}
