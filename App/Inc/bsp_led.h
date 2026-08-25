/**
 * @file    bsp_led.h
 * @brief   状态 LED 控制接口。
 */

#ifndef __BSP_LED_H
#define __BSP_LED_H

/**
 * @brief   初始化状态 LED。
 * 
 * 将 LED 设置为默认熄灭状态。
 * GPIO Mode，Output Type 等底层配置由 CubeMX 生成的 MX_GPIO_Init() 完成。
 */
void BSP_LED_Init(void);

/**
 * @brief 点亮状态 LED。
 */
void BSP_LED_On(void);

/**
 * @brief 熄灭状态 LED。
 */
void BSP_LED_Off(void);

/**
 * @brief 翻转状态 LED 当前输出状态。
 */
void BSP_LED_Toggle(void);

#endif
