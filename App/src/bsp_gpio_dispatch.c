/**
 * @file    bsp_gpio_dispatch.c
 * @brief   GPIO EXTI 中断事件分发实现。
 */

#include "bsp_gpio_dispatch.h"
#include "bsp_icm42688.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /*
     * STM32 HAL 将所有 GPIO EXTI 最终汇入该 Callback。
     * 这里仅根据 Pin 判断中断来源，再转交给对应设备的 ISR 处理入口。
     * 
     * GPIO_PIN_4 -> IMU1 Data Ready
     * GPIO_PIN_15 -> IMU2 Data Ready
     */
    switch (GPIO_Pin)
    {
        case GPIO_PIN_4:
            ICM_OnDataReady_ISR(ICM_INSTANCE_1);
            break;

        case GPIO_PIN_15:
            ICM_OnDataReady_ISR(ICM_INSTANCE_2);
            break;

        default:
            break;
    }
}
