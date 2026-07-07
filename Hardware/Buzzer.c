#include "Buzzer.h"
#include "Delay.h"
#include "protection.h"

/**
  * @brief  初始化 Buzzer TIM3Ch2 引脚
  */
void BUZZER_Init(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
	
	TIM_TimeBaseInitTypeDef tim;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 41045;		/* ARR */
	tim.TIM_Prescaler = 0;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &tim);
	
	TIM_OCInitTypeDef oc;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputNState = TIM_OutputNState_Disable;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 0;		/* 初始化不响，需要用到时再控制 */
	
	TIM_OC2Init(TIM3, &oc);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  开蜂鸣器
  */
void BUZZER_Start(){
	TIM_SetCompare2(TIM3, 20523);			// 20523
}

/**
  * @brief  关蜂鸣器
  */
void BUZZER_Stop(){
	TIM_SetCompare2(TIM3, 0);
}

/**
  * @brief  根据当前系统状态驱动蜂鸣器发声节奏
  * @note   使用底层 SysTick_ms (系统绝对时间) 划定固定的鸣叫周期与占空比，与 Main Loop 的调度频率解耦。
  * 必须保证 Main Loop 对该函数的调度周期小于最短的发声窗口 (200ms)，否则会导致漏鸣。
  * @param  None
  * @retval None
  */
void Buzzer_Drive(void)
{
    uint32_t phase;
    uint8_t on = 0;

    switch(buzzer_mode)
    {
        case BUZZ_NORMAL:                   // 正常状态下，3s 一响 (鸣叫 200ms)
            phase = SysTick_ms % 3000;
            on = (phase < 200);
            break;

        case BUZZ_LOW_VOLTAGE:              // 低电压状态下，0.4s 一响 (鸣叫 200ms)
            phase = SysTick_ms % 400;
            on = (phase < 200);
            break;

        case BUZZ_OVERCURRENT:              // 过流状态下，1s 一响 (鸣叫 500ms)
            phase = SysTick_ms % 1000;
            on = (phase < 500);
            break;

        case BUZZ_DISARMING:                // 未握手状态下，2s 一响 (鸣叫 1000ms，用于手动停机反馈)
            phase = SysTick_ms % 2000;
            on = (phase < 1000);
            break;
    }

    if(on) {
        BUZZER_Start();
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
    } else {
        BUZZER_Stop();
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
}