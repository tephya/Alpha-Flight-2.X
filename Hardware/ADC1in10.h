#ifndef __ADC1IN10_H
#define __ADC1IN10_H

#include "stm32f4xx.h"

extern float vbat;

void Init_ADC1(void);
void ReadVBAT(void);

#endif