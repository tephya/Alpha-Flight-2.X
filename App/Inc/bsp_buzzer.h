/**
 * @file    bsp_buzzer.h
 * @file    无源蜂鸣器 PWM 驱动接口。
 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

/**
 * @brief   初始化风名气 PWM 输出参数。
 * 
 * 根据 TIM3 实际时钟动态计算 ARR / CCR，
 * 使 PWM 频率尽量接近目标蜂鸣器频率。
 */
void BSP_Buzzer_Init(void);

/**
 * @brief   启动蜂鸣器 PWM 输出。
 */
void BSP_Buzzer_On(void);

/**
 * @brief   停止蜂鸣器 PWM 输出。
 */
void BSP_Buzzer_Off(void);

#endif
