/**
 * @file    bsp_buzzer.c
 * @brief   给予 TIM3 PWM 的无源蜂鸣器驱动实现。
 */

#include "bsp_buzzer.h"
#include "tim.h"

/** 蜂鸣器目标 PWM 频率，Hz。 */
#define BUZZER_FREQ_HZ 2048U

/**
 * PWM Duty Cycle，%。
 * 
 * 对无源蜂鸣器而言，Duty Cycle 会影响驱动波形及实际响度；
 * 当前统一通过此宏调整。
 */
#define BUZZER_DUTY_PERCENT 25U

extern TIM_HandleTypeDef htim3;

void BSP_Buzzer_Init(void)
{
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t flash_latency;

    /*
     * TIM3 挂载在 APB1。
     *
     * STM32F4 中，当 APB1 Prescaler 为 1 时：
     * TIM Clock = PCLK1。
     *
     * 当 APB1 Prescaler 不为 1 时：
     * TIM Clock = 2 × PCLK1。
     */
    HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);

    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();

    if(clk_cfg.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        tim_clk *= 2U;
    }

    /*
     * PSC 固定为 0，因此 Timer Counter Clock 等于 TIM3 Clock。
     * 
     * PWM Frequency：
     * 
     * f_pwm = tim_clk / (ARR + 1)
     * 
     * 所以目标 Period Count 为：
     *  
     * ARR + 1 = tim_clk / BUZZER_FREQ_HZ
     */
    uint32_t period_ticks = tim_clk / BUZZER_FREQ_HZ;

    /*
     * 防止极端时钟配置导致 Period 小于 2 Count，
     * 避免生成无意义的 ARR / CCR 参数。
     */
    if(period_ticks < 2U)
        period_ticks = 2U;

    TIM3->PSC = 0U;
    TIM3->ARR = (uint16_t)(period_ticks - 1U);

    /*
     * PWM Mode 下 CCR2 决定 High Level 所占 Period 比例。
     */
    TIM3->CCR2 = (uint16_t)(period_ticks * BUZZER_DUTY_PERCENT/ 100U);

    /*
     * 产生一次 Update Event，
     * 使新写入的 PSC / ARR 等预装载参数立即生效，
     * 不等待下一自然 Update Event。
     */
    TIM3->EGR = TIM_EGR_UG;
}

void BSP_Buzzer_On(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void BSP_Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}
