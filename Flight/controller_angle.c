#include "controller_angle.h"

AngleController_t angle_controller;

/**
  * @brief 	初始化角度控制器 （PID外环）
  */
void AngleController_Init(void)
{
	/*
     * 控制流向说明：
     * [输入] 角度误差 Error (单位: °) = 期望姿态角(Target) - IMU解算姿态角(Meas)
     * [输出] 期望角速度 (Rate Target, 单位: °/s)，送入内环(Rate Controller)作为目标值
     */

    PID_Init(&angle_controller.roll,
             2.5f,
             0.0f,
             0.0f,
             250.0f);

    PID_Init(&angle_controller.pitch,
             2.5f,
             0.0f,
             0.0f,
             250.0f);

    angle_controller.roll.target = 0.0f;
    angle_controller.pitch.target = 0.0f;

    angle_controller.roll_rate_target = 0.0f;
    angle_controller.pitch_rate_target = 0.0f;
}

/**
  * @brief	复位角度控制器 （PID外环）
  */
void AngleController_Reset(void)
{
    PID_Reset(&angle_controller.roll);
    PID_Reset(&angle_controller.pitch);

    angle_controller.roll_rate_target = 0.0f;
    angle_controller.pitch_rate_target = 0.0f;
}

/**
  * @brief	更新角度控制器（PID外环）
  * @param	roll_target		遥控器发出的 roll 目标值
  * @param	pitch_target	遥控器发出的 pitch 目标值
  * @param	roll			roll 实际测量值
  * @param	pitch			pitch 实际测量值
  * @param	dt				更新时间间隔
  */
void AngleController_Update(float roll_target,
                            float pitch_target,
                            float roll,
                            float pitch,
                            float dt)
{
    PID_SetTarget(&angle_controller.roll,
                  roll_target);

    PID_SetTarget(&angle_controller.pitch,
                  pitch_target);

    angle_controller.roll_rate_target =
        PID_Update(&angle_controller.roll,
                   roll,
                   dt);

    angle_controller.pitch_rate_target =
        PID_Update(&angle_controller.pitch,
                   pitch,
                   dt);
}