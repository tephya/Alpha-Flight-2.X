#include "controller_rate.h"

RateController_t rate_controller;

/**
  * @brief 	初始化角速度控制器 （PID内环）
  */
void RateController_Init(void)
{
	/*
     * 控制流向说明：
     * [输入] 角速度误差 Error (单位: °/s) = 期望角速度(Rate Target) - 陀螺仪实测角速度(Gyro)
     * [输出] 无量纲控制指令 (Mixer Command)，送入混控矩阵直接映射为电机功率差
     */

    PID_Init(&rate_controller.roll,
             1.00f,
             0.05f,
             0.015f,
             250.0f);

    PID_Init(&rate_controller.pitch,
             1.00f,
             0.05f,
             0.015f,
             250.0f);

    PID_Init(&rate_controller.yaw,
             2.00f,
             0.10f,
             0.00f,
             200.0f);
}

/**
  * @brief	复位角速度控制器 （PID内环）
  */
void RateController_Reset(void)
{
    PID_Reset(&rate_controller.roll);
    PID_Reset(&rate_controller.pitch);
    PID_Reset(&rate_controller.yaw);

    rate_controller.roll_output = 0.0f;
    rate_controller.pitch_output = 0.0f;
    rate_controller.yaw_output = 0.0f;
}

/**
  * @brief	更新角速度控制器（PID内环）
  * @param	roll_rate_target		外环解算出的 roll 变化速率 目标值
  * @param	pitch_rate_target		外环解算出的 pitch 变化速率 目标值
  * @param	yaw_rate_target			yaw 变化速率 目标值
  * @param	roll_rate				roll 实际 角速度， 即GyroX
  * @param	pitch_rate				pitch 实际 角速度， 即GyroY
  * @param	yaw_rate				yaw 实际 角速度， 即GyroZ
  * @param	dt						更新时间间隔
  */
void RateController_Update(float roll_rate_target,
                           float pitch_rate_target,
                           float yaw_rate_target,
                           float roll_rate,
                           float pitch_rate,
                           float yaw_rate,
                           float dt)
{
    PID_SetTarget(&rate_controller.roll,
                  roll_rate_target);

    PID_SetTarget(&rate_controller.pitch,
                  pitch_rate_target);

    PID_SetTarget(&rate_controller.yaw,
                  yaw_rate_target);

    rate_controller.roll_output =
        PID_Update(&rate_controller.roll,
                   roll_rate,
                   dt);

    rate_controller.pitch_output =
        PID_Update(&rate_controller.pitch,
                   pitch_rate,
                   dt);

    rate_controller.yaw_output =
        PID_Update(&rate_controller.yaw,
                   yaw_rate,
                   dt);
}