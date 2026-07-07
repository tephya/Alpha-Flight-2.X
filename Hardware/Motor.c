#include "Motor.h"

void Motor_PWM_Init(void){
	// 使能GPIO时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	TIM_TimeBaseInitTypeDef TimeBaseInitStructure;
	TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimeBaseInitStructure.TIM_CounterMode =	TIM_CounterMode_Up;
	
	/* APB2 Clock is 168MHz and ESC is 50 Hz;
	 * 168MHz / (Prescaler + 1)*(Period + 1) = 50Hz;
	 *  Then (Prescaler + 1)*(Period + 1) = 3 360 000
	 * Set (Prescaler + 1) = 168;
	 * 	Then (Period + 1) = 20000
	 *	which means that '1 Period' == '20ms(50Hz)' == '20000 clocks'
	 * &'1ms' == '1000 clocks'
	 */
	TimeBaseInitStructure.TIM_Period = 19999;
	TimeBaseInitStructure.TIM_Prescaler = 167;					
	TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitTypeDefStructure;
	TIM_OCInitTypeDefStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitTypeDefStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitTypeDefStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitTypeDefStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitTypeDefStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitTypeDefStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitTypeDefStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitTypeDefStructure.TIM_Pulse = 1000;
	
	TIM_OC1Init(TIM1, &TIM_OCInitTypeDefStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitTypeDefStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitTypeDefStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitTypeDefStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);		// MOE使能
}

void Motor_SetSpeed(uint8_t motor, uint16_t pulse_us){
	/* CCR: Capture Compare Register whick is compared with current
	 *	clock ([0,19999]) by timer;
	 */
	// 1ms == 1000clocks --> 1us == 1clock;
	// CCR sets the number of clock cycles the output stays HIGH
	uint16_t CCR = pulse_us;
	switch(motor){
		case 1: 
			TIM_SetCompare1(TIM1, CCR);
			break;
		case 2:
			TIM_SetCompare2(TIM1, CCR);
			break;
		case 3:
			TIM_SetCompare3(TIM1, CCR);
			break;
		case 4:
			TIM_SetCompare4(TIM1, CCR);
			break;
		default:
			break;
			// error
	}
}