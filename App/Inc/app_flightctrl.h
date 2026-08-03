#ifndef __APP_FLIGHTCTRL_H
#define __APP_FLIGHTCTRL_H

#include <stdint.h>

typedef struct
{
    float roll_target, pitch_target; // dge, 来自遥控映射
    float yaw_target;                // deg, Heading Hold锁定的目标航向
    uint8_t yaw_mode;                // 0=Heading Hold, 1=手动速率
    uint8_t airmode_active;         // 0=起飞前抑制姿态输出，1=低油门仍保持姿态控制

    float yaw_rate_target;                     // deg/s
    float roll_rate_target, pitch_rate_target; // deg/s, AngleController输出

    float roll_cmd, pitch_cmd, yaw_cmd; // RateController输出，送入Mixer做desaturation

    uint16_t throttle;

    float roll_meas, pitch_meas, yaw_meas; // deg, 来自attitude融合结果换算
} FlightControl_t;


void App_FlightCtrl_Task(void *argument);
void FlightControl_CopyTo(FlightControl_t *out); /* Task_Blackbox等外部消费者用这个拿快照，不直接碰内部fc */

#endif
