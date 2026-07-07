#ifndef __DSHOT4_H
#define __DSHOT4_H

#include "stm32f4xx.h"
#include <stdint.h>

void DSHOT4_Init(void);
void DSHOT4_Send(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4);
static uint16_t thr_to_dshot(uint16_t thr, uint8_t arm_state);

#endif