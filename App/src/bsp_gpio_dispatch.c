#include "bsp_gpio_dispatch.h"
#include "bsp_icm42688.h"

/**
 * @brief   重写外部中断汇入仲裁函数，根据引脚开启对应的中断服务
 * @param   GPIO_Pin    中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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
