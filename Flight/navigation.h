#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "stm32f4xx.h"

typedef enum {
	RTH_IDLE,				// 空闲状态
	RTH_YAW_TOHOME,			// 自动偏航对准至home点状态
	RTH_FLY_HOME,			// 自动返航状态
	RTH_HOVER				// 悬停状态
} RTH_State_t;

extern RTH_State_t rth_state;
extern uint16_t hover_throttle;		// 首飞后标定

extern float bearing;
extern float dist;

float GPS_DistanceTo(float lat, float lon);		// 当前位置到目标点的距离(米)
float GPS_BearingTo(double lat, double lon);         // 当前位置到目标点的航向（度，真北0°顺时针）

void RTH_Update(float bearing, float dist, float yaw_deg, float *target_roll,
				float *target_pitch, float *target_yaw, volatile uint16_t *throttle);
uint8_t RTH_IsActive(void);
void RTH_Trigger(void);
void RTH_Cancel(void);

#endif
