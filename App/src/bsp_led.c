/**
 * @file    bsp_led.c
 * @brief   状态 LED GPIO 控制实现。
 */

#include "bsp_led.h"
#include "gpio.h"

/*
 * LED 有效点评定义。
 * 
 * 当前硬件为 High Active：
 * GPIO Set     -> LED On
 * GPIO Reset   -> LED Off
 */
#define LED_ON_LEVEL GPIO_PIN_SET
#define LED_OFF_LEVEL GPIO_PIN_RESET

void BSP_LED_Init(void)
{
    /*
     * GPIO 本身已经由 MX_GPIO_Init() 完成配置，
     * 此处只设置 LED 的初始输出状态，避免重复初始化 GPIO。
     */
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

