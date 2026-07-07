#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H

#include "stm32f4xx.h"

typedef struct
{
    /* 感知测量 (Measurements) */
    float roll_meas;           // 横滚角实测值 (°)
    float pitch_meas;          // 俯仰角实测值 (°)
    float yaw_meas;            // 航向角实测值 (°)

    /* 期望目标 (Targets - 摇杆输入映射) */
    float roll_target;         // 期望横滚角 (°)
    float pitch_target;        // 期望俯仰角 (°)
    float yaw_target;          // 期望航向角 (°) - 仅在航向锁定模式下更新
    uint16_t throttle;         // 期望油门值 (0-1000)

    /* 角速度目标 (Rate Targets - 串级PID中间量) */
    float roll_rate_target;    // Roll 外环输出的期望角速度 (°/s)
    float pitch_rate_target;   // Pitch 外环输出的期望角速度 (°/s)
    float yaw_rate_target;     // Yaw 期望角速度 (°/s) - Rate模式下直接来自摇杆，锁定模式下来自外环

    /* 执行指令 (Commands - 送入混控器) */
    float roll_cmd;            // Roll 内环输出的控制律 (无量纲)
    float pitch_cmd;           // Pitch 内环输出的控制律 (无量纲)
    float yaw_cmd;             // Yaw 内环输出的控制律 (无量纲)
	
    /* 系统状态 (System State) */
    uint8_t yaw_mode;          // 0 = ANGLE_LOCK (航向锁定), 1 = RATE (手动偏航)

} FlightControl_t;

extern FlightControl_t fc;

void FlightController_Init(void);
void FlightController_Reset(void);
void FlightController_Update(float dt, float gyro_x, float gyro_y, float gyro_z);
void FlightController_UpdateYaw(float yaw_deg);

float Map_Roll(uint16_t ch);     // CRSF 172~1811 → -30.0° ~ +30.0°
float Map_Pitch(uint16_t ch);    // CRSF 172~1811 → -30.0° ~ +30.0°
float Map_Yaw(uint16_t ch);      // CRSF 172~1811 → -200.0°/s ~ +200.0°/s
uint16_t Map_Throttle(uint16_t ch); // CRSF 172~1811 → DSHOT 0 ~ 1000

#endif