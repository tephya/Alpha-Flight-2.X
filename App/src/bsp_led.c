#include "bsp_led.h"
#include "gpio.h"

#define LED_ON_LEVEL GPIO_PIN_SET
#define LED_OFF_LEVEL GPIO_PIN_RESET

/**
 * @brief   LED初始化，只负责置默认熄灭状态
 * @note    PB9本身的GPIO_Output/Push-Pull配置由CubeMX的MX_GPIO_Init()完成
 *          这里不重复做Init，避免跟CubeMX生成的配置冲突
 */
void BSP_LED_Init(void)
{
    HAL_GPIO_WritePin(State_LED_GPIO_Port, State_LED_Pin, LED_OFF_LEVEL);
}

void BSP_LED_On(void)
{
    HAL_GPIO_WritePin(State_LED_GPIO_Port, State_LED_Pin, LED_ON_LEVEL);
}

void BSP_LED_Off(void)
{
    HAL_GPIO_WritePin(State_LED_GPIO_Port, State_LED_Pin, LED_OFF_LEVEL);
}

void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(State_LED_GPIO_Port, State_LED_Pin);
}

