#include "bsp_buzzer.h"
#include "tim.h"

#define BUZZER_FREQ_HZ 2048U // 无源蜂鸣器谐振频率2.048KHz(根据实际情况调整)
#define BUZZER_DUTY_PERCENT 25U // 音量大小调整入口(50U时音量最大，左右递增或递减音量会削减)

extern TIM_HandleTypeDef htim3;

/**
 * @brief   动态计算TIM3实际时钟频率，据此算出ARR/CCR使输出精确落在BUZZER_FREQ_HZ
 * @note    TIM3挂载APB1总线上：若APB1预分频≠1，其定时器时钟是APB1总线时钟的2倍
 *          (参考手册固定规则，跟具体分频值是多少无关，无需假设CubeMX怎么配的)。
 *          PSC固定取0——TIM3是16位定时器(ARR上限65535)，以F405上TIM3实际可能的最高时钟(84MHz)
 *          计算，84MHz/2048Hz=41016，小于65535，PSC=0足够，不需要额外分频。
 */
void BSP_Buzzer_Init(void)
{
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);

    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if(clk_cfg.APB1CLKDivider != RCC_HCLK_DIV1)
        tim_clk *= 2U;      // APB1预分频不为1时，挂在APB1上的定时器时钟是APB1总线时钟的2倍

    uint32_t period_ticks = tim_clk / BUZZER_FREQ_HZ; // PSC=0时，ARR+1需要等于这个值
    if(period_ticks < 2U)
        period_ticks = 2U;  // 极端情况下的极限保护，避免ARR=0导致PWM异常

    TIM3->PSC = 0U;
    TIM3->ARR = (uint16_t)(period_ticks - 1U);
    TIM3->CCR2 = (uint16_t)(period_ticks * BUZZER_DUTY_PERCENT/ 100U);

    /**
     * 手动产生一次Update事件，把上面写的PSC/ARR/CCR立即从预装载寄存器刷进影子寄存器生效
     * 不等到定时器自然跑完一圈才生效
     */
    TIM3->EGR = TIM_EGR_UG;
}

/**
 * @brief   启动蜂鸣器发声(持续输出2.048kHz方波)
 */
void BSP_Buzzer_On(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/**
 * @brief   停止蜂鸣器发声
 */
void BSP_Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}
