/**
 * @file    bsp_dshot.h
 * @brief   DShot600 四路电机输出接口。
 */

#ifndef __BSP_DSHOT_H
#define __BSP_DSHOT_H

#include <stdint.h>

/**
 * @brief   初始化 DShot600 四路 PWM 输出。
 * 
 * 启动 TIM1 CH1~CH4，并初始化 DMA Burst Buffer。
 */
void BSP_DSHOT_Init(void);

/**
 * @brief   编码并发送一帧四路DShot600 数据。
 */
void BSP_DSHOT_Send(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4);

#endif
