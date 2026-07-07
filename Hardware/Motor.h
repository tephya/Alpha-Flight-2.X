#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

void Motor_PWM_Init(void);
void Motor_SetSpeed(uint8_t motor, uint16_t pulse_us);

#endif
