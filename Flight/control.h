#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "PID.h"
#include "flight_controller.h"
#include <stdbool.h>

typedef struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
} Motor_Output_t;


extern Motor_Output_t motor_thr_data;

extern volatile uint8_t arm_state;		/* Disarmed | Armed */
extern uint8_t end_count;		/* Counting end_confirm loop */


void Control_SoftwareTask_Init(void);

uint16_t thr_to_dshot(uint16_t thr, uint8_t arm_state);
void Mixer(float roll_cmd,
           float pitch_cmd,
           float yaw_cmd,
           uint16_t throttle,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4);

bool IS_ARMED(void);
bool Thr_LDet(void);

#endif
