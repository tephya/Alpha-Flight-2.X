#ifndef __CONTROLLER_RATE_H
#define __CONTROLLER_RATE_H

#include "PID.h"

typedef struct
{
    PID_t roll;					/* （PID内环） roll控制变量 */
    PID_t pitch;				/* （PID内环） pitch控制变量 */
    PID_t yaw;					/* （PID内环） yaw控制变量 */

    float roll_output;			/* 内环解算出的输出到电机上的 roll 无量纲值 */
    float pitch_output;			/* 内环解算出的输出到电机上的 Pitch 无量纲值 */
    float yaw_output;			/* 内环解算出的输出到电机上的 yaw 无量纲值 */

} RateController_t;

extern RateController_t rate_controller;

void RateController_Init(void);

void RateController_Reset(void);

void RateController_Update(float roll_rate_target,
                           float pitch_rate_target,
                           float yaw_rate_target,
                           float roll_rate,
                           float pitch_rate,
                           float yaw_rate,
                           float dt);

#endif