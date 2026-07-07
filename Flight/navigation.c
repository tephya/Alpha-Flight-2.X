#include "navigation.h"
#include "GPS.h"
#include <math.h>

#define DEG2RAD 0.017453293f
#define EARTH_R 6371000.0f		// 地球平均半径，米

RTH_State_t rth_state = RTH_IDLE;
uint16_t hover_throttle = 950;			// 临时值，飞稳后标定

float bearing = 0.0f;
float dist	  = 0.0f;

/**
  * @brief  计算当前位置到首飞点的距离
  * @param  lat		当前位置的纬度值
  * @param  lon		当前位置的经度值
  */
float GPS_DistanceTo(float lat, float lon)
{
	double dlat = (lat - gps_data.latitude) * DEG2RAD;
	double dlon = (lon - gps_data.longitude) * DEG2RAD;
	double lat1 = gps_data.latitude * DEG2RAD;
	double lat2 = lat * DEG2RAD;
	
	double a = sin(dlat/2) * sin(dlat/2)
			 + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);	
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	
	return (float)(EARTH_R * c);
}

/**
  * @brief  计算当前位置到首飞点的偏航值
  * @param  lat		当前位置的纬度值
  * @param  lon		当前位置的经度值
  */
float GPS_BearingTo(double lat, double lon)
{
	double lat1 = gps_data.latitude * DEG2RAD;
	double lat2 = lat * DEG2RAD;
	double dlon = (lon - gps_data.longitude) * DEG2RAD;

	double y = sin(dlon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
	float bearing = atan2(y, x) * 57.2958f;
	
	if(bearing < 0) bearing += 360.0f;			// 归一化到0~360
	return bearing;
}

/**
  * @brief  触发自动返航 (RTH) 序列
  * @note   仅在 RTH 状态机处于空闲 (IDLE) 时有效，触发后进入机头对准 (YAW_TOHOME) 阶段
  */
void RTH_Trigger(void)
{
	if(rth_state == RTH_IDLE)
		rth_state = RTH_YAW_TOHOME;
}

/**
  * @brief  取消自动返航，交回控制权
  * @note   强制将 RTH 状态机重置为空闲状态 (IDLE)
  */
void RTH_Cancel(void){
	rth_state = RTH_IDLE;
}

/**
  * @brief  检查 RTH 功能当前是否处于激活接管状态
  * @retval 1 (true) : RTH 正在执行中 (在忙)
  * @retval 0 (false): RTH 处于空闲状态 (未激活)
  */
uint8_t RTH_IsActive(void){
    return rth_state != RTH_IDLE;
}


/**
  * @brief  自动返航 (RTH) 核心状态机更新函数
  * @note   [WARNING] 危险/实验性代码：当前仍为开环控制（定高和定距缺乏闭环 PID）。
  * 外场实飞前，必须结合气压计定高环与 GPS 速度位置环进行重构验证！
  * * @param  bearing       目标 Home 点的绝对航向角 (°)
  * @param  dist          当前距离 Home 点的平面距离 (m)
  * @param  yaw_deg       当前无人机的真实航向角 (°)
  * @param  target_roll   [输出] 期望横滚角指针
  * @param  target_pitch  [输出] 期望俯仰角指针
  * @param  target_yaw    [输出] 期望航向角指针
  * @param  throttle      [输出] 期望油门值指针
  */
void RTH_Update(float bearing, float dist, float yaw_deg, 
                float *target_roll, float *target_pitch, float *target_yaw, 
                volatile uint16_t *throttle)
{
    if(rth_state == RTH_IDLE) return;
    
    // [v2] 优化：已将 bearing 和 dist 的大尺度浮点运算解耦到 Main Loop 的低频任务中计算，
    // 防止占用姿态解算/控制环的极高优先级算力，缓和返航过程。
    float yaw_err = bearing - yaw_deg;
    
    // 归一化偏差到 -180° ~ +180° 区间
    if(yaw_err > 180.0f) yaw_err -= 360.0f;
    if(yaw_err < -180.0f) yaw_err += 360.0f;
    
    // 返航全局基础设定：强制回平横滚，并使用预设悬停油门
    *throttle = hover_throttle;
    *target_roll = 0.0f;
    
    switch(rth_state){
        case RTH_YAW_TOHOME:
            /* 阶段 1：原地旋转，机头对准 Home 点 */
            *target_pitch = 0.0f;
            *target_yaw = yaw_deg + yaw_err; // 追踪目标航向
            
            if(fabsf(yaw_err) < 10.0f)       // 误差小于 10 度时，进入下一阶段
                rth_state = RTH_FLY_HOME;
            break;
            
        case RTH_FLY_HOME:
            /* 阶段 2：定高直线飞向 Home 点 */
            *target_pitch = -8.0f;           // 机头下压 8 度产生前进分力
            *target_yaw = yaw_deg + yaw_err; // 飞行过程中持续修正航向抗风
            
            if(dist < 5.0f)                  // 距离 Home 点小于 5 米，进入悬停
                rth_state = RTH_HOVER;
            break;
            
        case RTH_HOVER:
            /* 阶段 3：到达 Home 点上方，原地悬停等待接管 */
            *target_pitch = 0.0f;
            *target_yaw = yaw_deg;           // 锁定当前航向，不再转动
            break;
            
        default:
            break;
    }
}