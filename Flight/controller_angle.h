#ifndef __CONTROLLER_ANGLE_H
#define __CONTROLLER_ANGLE_H

#include "PID.h"

typedef struct
{
    PID_t roll;						/* （PID外环） roll控制变量 */
    PID_t pitch;					/* （PID外环） pitch控制变量 */

    float roll_rate_target;			/* （PID内环） roll 角速度目标值 */
    float pitch_rate_target;		/* （PID内环） pitch 角速度目标值 */

} AngleController_t;

extern AngleController_t angle_controller;

void AngleController_Init(void);

void AngleController_Reset(void);

void AngleController_Update(float roll_target,
                            float pitch_target,
                            float roll,
                            float pitch,
                            float dt);

#endif