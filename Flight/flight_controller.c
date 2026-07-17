#include "flight_controller.h"

#include "elrs.h"
#include "ICM_42688P.h"

#include "controller_angle.h"
#include "controller_rate.h"
#include "protection.h"
#include "navigation.h"

#include <math.h>

/* * ==================================================================
 * [EXPERIMENTAL FEATURES] 实验性功能总开关
 * 警告：未经过充分外场测试前，严禁将以下宏定义置为 1！
 * ==================================================================
 */
#define ENABLE_AUTO_RTH    0   // 自动返航功能 (0: 关闭物理调用, 1: 开启)

#define THROTTLE_MAX 1152.0f

FlightControl_t fc;

/* =========================================================================
 * 【架构说明】关于独立保留 pid_yaw (偏航角外环控制器) 的设计考量
 * =========================================================================
 * 为什么不使用 fc (FlightControl_t) 直接进行偏差计算来替代 pid_yaw？
 * * 1. 状态记忆刚需 (State Memory)：
 * fc 仅为实时状态容器，无记忆功能。航向锁定 (Heading Hold) 依赖 PID 的积分项 (I) 
 * 来抵抗持续性的外部扰动（如机架重心偏置、侧风干扰）。pid_yaw 实例化对象负责存储
 * 误差积分 (Integral) 和历史状态，这是维持航向不随时间缓慢漂移的物理基石。
 * * 2. 轴逻辑解耦 (Logic Decoupling)：
 * Yaw 轴具有的特殊性：不仅存在特有的 [-180°, +180°] 过零点环绕突变问题，
 * 还需在 手动角速度模式 (Rate Mode) 和 航向锁定模式 (Angle Lock) 之间进行无缝切换。
 * 将其从常规的 Roll/Pitch 姿态外环 (AngleController) 中剥离并独立实例化，
 * 能最大程度保证控制链路的清晰与系统的鲁棒性。
 * ========================================================================= */
PID_t pid_yaw;

/**
  * @brief 	初始化飞行控制器
  */
void FlightController_Init(void)
{
    AngleController_Init();
    RateController_Init();

    PID_Init(&pid_yaw, 0.0f, 0.0f, 0.0f, 200.0f);

    fc.yaw_mode = 0;
    fc.yaw_target = 0.0f;
}

/**
  * @brief 	复位飞行控制器
  */
void FlightController_Reset(void)
{
    AngleController_Reset();
    RateController_Reset();

    fc.roll_target = 0.0f;
    fc.pitch_target = 0.0f;
    fc.yaw_rate_target = 0.0f;

    fc.roll_rate_target = 0.0f;
    fc.pitch_rate_target = 0.0f;

    fc.roll_cmd = 0.0f;
    fc.pitch_cmd = 0.0f;
    fc.yaw_cmd = 0.0f;

    fc.throttle = 0;

    fc.roll_meas = 0.0f;
    fc.pitch_meas = 0.0f;
    fc.yaw_meas = 0.0f;
	
	fc.yaw_target = fc.yaw_meas;
	fc.yaw_mode = 0;

	PID_Reset(&pid_yaw);
}

/**
  * @brief	更新飞行控制器
  * @param	dt				更新时间间隔
  * @param	gyro_x			当前的 Roll 变化角速度，即 X 轴角速度
  * @param	gyro_y			当前的 Pitch 变化角速度，即 Y 轴角速度
  * @param	gyro_z			当前的 Yaw 变化角速度，即 Z 轴角速度
  */
void FlightController_Update(float dt, float gyro_x, float gyro_y, float gyro_z)
{
/*================ Target Generation MUX (指令多路复用) ================*/
	if(rth_state == RTH_IDLE){
		/*================ RC Mapping ================*/

		fc.roll_target  = Map_Roll(crsf_data.channels[0]);
		fc.pitch_target = Map_Pitch(crsf_data.channels[1]);
		fc.throttle     = Map_Throttle(crsf_data.channels[2]);

		/*================ Yaw =======================*/

		if(crsf_data.channels[5] > 1500)
		{
			/* 手动偏航状态 */
			fc.yaw_mode = 1;

			fc.yaw_rate_target = Map_Yaw(crsf_data.channels[3]);

			if(fabsf(fc.yaw_rate_target) < 5.0f)
				fc.yaw_rate_target = 0.0f;
		}
		else
		{
			/* 第一次进入 Heading Hold，锁定当前航向 */
			if(fc.yaw_mode)
			{
				fc.yaw_target = fc.yaw_meas;
				pid_yaw.integral = 0.0f;
			}

			fc.yaw_mode = 0;

			float yaw_err = fc.yaw_target - fc.yaw_meas;

			if(yaw_err > 180.0f)
				yaw_err -= 360.0f;

			if(yaw_err < -180.0f)
				yaw_err += 360.0f;

			pid_yaw.target = 0.0f;
			
			/* Heading Hold状态下，修正航向为初始航向 */
			fc.yaw_rate_target = PID_Update(&pid_yaw, -yaw_err,	dt);
		}
	}
	else{
		/* 状态 2：机器接管模式 (RTH 返航) */
        
        // [P1]：什么都不做！不读取摇杆数据。
        // fc.roll_target, fc.pitch_target, fc.throttle 等已经在 
        // Main Loop 的 RTH_Update() 中被赋好值了，这里直接继承使用。

        // [P2]：强行覆盖 Yaw 的工作模式
        // 返航时必须是“航向锁定（Heading Hold）”模式，指向 Home 点
        fc.yaw_mode = 0; 
        
        float yaw_err = fc.yaw_target - fc.yaw_meas; // 这里的 yaw_target 也是 RTH 给的
        if(yaw_err > 180.0f)  yaw_err -= 360.0f;
        if(yaw_err < -180.0f) yaw_err += 360.0f;
        
        pid_yaw.target = 0.0f;
        fc.yaw_rate_target = PID_Update(&pid_yaw, -yaw_err, dt);
	}


    /*================ Angle Controller =================*/

    AngleController_Update(
        fc.roll_target,
        fc.pitch_target,
        fc.roll_meas,
        -fc.pitch_meas,			// 加上“-”才能让 IMU 的测量轴符合 NED 坐标轴
        dt);

    fc.roll_rate_target  = angle_controller.roll_rate_target;
    fc.pitch_rate_target = angle_controller.pitch_rate_target;

    /*================ Rate Controller =================*/

	RateController_Update(
			fc.roll_rate_target,
			fc.pitch_rate_target,
			fc.yaw_rate_target,			// Yaw Rate Target
			gyro_x,
			-gyro_y,			// 加上“-”才能让 IMU 的测量轴符合 NED 坐标轴
			gyro_z,
			dt);

    /*================ Controller Output =================*/

    fc.roll_cmd  = rate_controller.roll_output;
    fc.pitch_cmd = rate_controller.pitch_output;
    fc.yaw_cmd   = rate_controller.yaw_output;
	
	/*================ Dynamic Limit =================*/

	fc.throttle = (fc.throttle > throttle_limit)
				? throttle_limit
				: fc.throttle;

	if(fc.throttle < 30)
	{
		FlightController_Reset();
	}
	else
	{
		float dynamic_limit = (float)(fc.throttle - 30);

		if(dynamic_limit > 250.0f)
			dynamic_limit = 250.0f;

		if(fc.roll_cmd > dynamic_limit)
			fc.roll_cmd = dynamic_limit;
		if(fc.roll_cmd < -dynamic_limit)
			fc.roll_cmd = -dynamic_limit;

		if(fc.pitch_cmd > dynamic_limit)
			fc.pitch_cmd = dynamic_limit;
		if(fc.pitch_cmd < -dynamic_limit)
			fc.pitch_cmd = -dynamic_limit;

		if(fc.yaw_cmd > dynamic_limit)
			fc.yaw_cmd = dynamic_limit;
		if(fc.yaw_cmd < -dynamic_limit)
			fc.yaw_cmd = -dynamic_limit;
	}
}

/**
  * @brief 	更新飞行控制器的航向角测量值
  * @param	测量的航向角的值
  */
void FlightController_UpdateYaw(float yaw_deg)
{
    fc.yaw_meas = yaw_deg;
}

/**
  * @brief 	匹配 Roll 遥控杆的通道值 到期望角度
  * @note	匹配范围： -20° ~ 20°
  * @param	Roll ELRS 通道值
  * @retval 映射后的目标横滚角速度，单位: °/s
  */
float Map_Roll(uint16_t ch){
	return (ch - 991.5f) / 819.5f * 20.0f;
}

/**
  * @brief 	匹配 Pitch 遥控杆的通道值 到期望角度
  * @note	匹配范围： -20° ~ 20°
  * @param	Pitch ELRS 通道值
  * @retval 映射后的目标俯仰角速度，单位: °/s
  */
float Map_Pitch(uint16_t ch){
	return -(ch - 991.5f) / 819.5f * 20.0f;
}

/**
  * @brief  匹配 Yaw 遥控杆的通道值 到期望角速度 (Rate)
  * @note	匹配范围： -90° ~ 90°
  * @param  ch  ELRS 通道值
  * @retval 映射后的目标偏航角速度，单位: °/s
  */
float Map_Yaw(uint16_t ch){
    return (ch - 991.5f) / 819.5f * 90.0f;
}

/**
  * @brief 	匹配 油门 遥控杆的通道值 到设定油门范围内的值
  * @note	匹配范围： 0 ~ 1152(max:2048)
  * @param	油门 ELRS 通道值
  * @retval 映射后的目标油门值
  */
uint16_t Map_Throttle(uint16_t ch){
    return (uint16_t)((ch - 172) / 1639.0f * THROTTLE_MAX);
}