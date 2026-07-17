#include "attitude.h"
#include <math.h>


Attitude_t att1 = {0};
Attitude_t att2 = {0};

static const float DEG_TO_RAD = 0.01745329252f;

/**
  * @brief  由加速度计原始数据计算Roll和Pitch角
  * @param  imu_data   IMU 数据结构体，存有 RAW 数据
  * @param  att    	   姿态解算数据结构体
  * @retval None
  */
void ICM_GetRollPitch(IMU_Data_t *imu_data, Attitude_t *att){
    // 欧拉角旋转顺序：先Roll后Pitch，Pitch分母需补偿Roll的影响
    att->accel_roll = atan2f(imu_data->ay, imu_data->az);
	float az = imu_data->az;
	float ay = imu_data->ay;
    att->accel_pitch = atan2f(-1* imu_data->ax, sqrtf(az * az + ay * ay));
}

/**
  * @brief  加速度计和陀螺仪互补滤波融合
  * @param  imu_data   IMU 数据结构体，存有 RAW 数据
  * @param  att    	   姿态解算数据结构体
  * @param  dt         融合滤波时间间隔，单位s
  * @retval None
  */
void Attitude_Update(IMU_Data_t *imu_data, Attitude_t *att, float dt)

{
    float GyroX, GyroY;
    // 将陀螺仪单位转为 rad/s
    GyroX = imu_data->gx * DEG_TO_RAD;
    GyroY = imu_data->gy * DEG_TO_RAD;

    att->roll  = GYRO_TRUST * (att->roll  + GyroX * dt) + (1.0f - GYRO_TRUST) * att->accel_roll;
    att->pitch = GYRO_TRUST * (att->pitch + GyroY * dt) + (1.0f - GYRO_TRUST) * att->accel_pitch;
}

/**
  * @brief  带倾斜补偿的Yaw角计算
  * @param 	qmc_data: 磁力计结构体
  * @param  att: 姿态角结构体
  * @retval None
  */
void Attitude_CptYaw(const QMC_Data_t *qmc_data, Attitude_t *att){
    float MX_h, MY_h;
    // 旋转矩阵 Rx(Roll)·Ry(Pitch)，将磁力计向量投影回水平面
    MX_h = cosf(att->pitch) * qmc_data->MX 
         + sinf(att->roll) * sinf(att->pitch) * qmc_data->MY 
         + cosf(att->roll) * sinf(att->pitch) * qmc_data->MZ;
    
    MY_h = cosf(att->roll) * qmc_data->MY 
         - sinf(att->roll) * qmc_data->MZ;
    
    att->yaw = atan2f(-MY_h, MX_h);
}