#include "Delay.h"

volatile uint32_t Tick = 0;					// ms级延时计时器，单位：0.1ms
volatile uint32_t SysTick_ms = 0;

/**
  * @brief  初始化SysTick，中断周期为0.1ms
  * @param	None
  * @retval None
  */
void SysTick_Init(void){
	SysTick_Config(SystemCoreClock / 10000);	// 每0.1ms触发一次中断
}

/**
  * @brief  ms级阻塞延时
  * @param	xms: 延时毫秒数
  * @retval None
  */
void Delay_ms(uint32_t xms){
	Tick = xms * 10;				// 1ms = 10个0.1ms中断周期
	while(Tick > 0);
}

/**
  * @brief  us级阻塞延时（基于SysTick->VAL轮询）
  * @param	xus: 延时微秒数
  * @retval None
  */
void Delay_us(uint32_t xus){
	uint32_t last = SysTick->VAL;		// 记录起始计数值
	uint32_t current = 0;
	uint32_t target = xus * 168;		// 168MHz/10000(0.1ms中断) 下，1us = 168个时钟周期
	uint32_t counter = 0;				// 已累计的时钟周期数
	
	while(counter < target){
		current = SysTick->VAL;
		if(last < current)				// VAL下溢后重装，处理溢出情况
			counter += last + (SysTick->LOAD - current);
		else
			counter += (last - current);
			
		last = current;					// 更新基准值
	}
}
		
void SysTick_Handler(void){
	static int cnt = 0;
	if(Tick > 0)
		Tick--;							// 每0.1ms递减一次
	if(++cnt >= 10){
		cnt = 0;
		SysTick_ms++;
	}
}	