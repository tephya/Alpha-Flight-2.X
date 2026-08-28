/**
 * @file    bsp_gpio_dispatch.h
 * @file    GPIO EXTI 中断分发接口。
 */

#ifndef __BSP_GPIO_DISPATCH_H
#define __BSP_GPIO_DISPATCH_H

#include "stm32f4xx_hal.h"

/**
 * @brief   HAL GPIO EXTI 统一回调入口。
 * 
 * 根据触发 EXTI 的 GPIO Pin，将中断事件分发给对应外设驱动。
 * 
 * @param[in] GPIO_Pin 触发 EXTI 的 GPIO Pin Mask。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
