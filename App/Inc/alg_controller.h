#ifndef __ALG_CONTROLLER_H
#define __ALG_CONTROLLER_H

#include <stdint.h>
#include "alg_pid.h"

// 遥控油门映射的量程上限
#define ALG_THROTTLE_MAX 1152.0f

/**
 * 角度控制器(PID外环)
 * 输入：角度误差(Target-Meas，单位°)
 * 输出：角速度目标值(单位°)，送入内环
 */
typedef struct
{
    PID_t roll;
    PID_t pitch;

    float roll_rate_target;
    float pitch_rate_target;
} AngleController_t;

extern AngleController_t angle_controller;

void AngleController_Init(void);
void AngleController_Reset(void);
void AngleController_Update(float roll_target,
                            float pitch_target,
                            float roll_meas,
                            float pitch_meas,
                            float dt);

/**
 * 角速度控制器(PID内环)
 * 输入：角速度误差(单位°/s)
 * 输出：无量纲控制指令，直接送混控
 */
typedef struct
{
    PID_t roll;
    PID_t pitch;
    PID_t yaw;

    float roll_output;
    float pitch_output;
    float yaw_output;
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

/**
 * 航向锁定误差计算，含[-180,180]过零点处理；
 * pid_yaw的target固定为0，函数内部算好误差取负值喂给PID_Update
 * 
 * 何时进入/退出该模式、pid_yaw->integral何时清零，属于业务状态机决策，
 * 由app_flightctrl.c负责，本函数只做纯计算，不持有pid_yaw实体
 */
float YawHeadingHold_Update(PID_t *pid_yaw,
                            float yaw_target_deg,
                            float yaw_meas_deg,
                            float dt);
                    
// 遥控通道值 -> 控制目标值 映射
float Map_Roll(uint16_t ch);
float Map_Pitch(uint16_t ch);
float Map_Yaw(uint16_t ch);
uint16_t Map_Throttle(uint16_t ch);

#endif
