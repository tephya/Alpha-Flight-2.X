#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f4xx.h"

extern volatile uint32_t SysTick_ms;

void SysTick_Init(void);
void Delay_ms(uint32_t xms);
void Delay_us(uint32_t xms);
void SysTick_Handler(void);

#endif
