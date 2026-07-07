#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"
#include "ICM_42688P.h"
#include "QMC5883P.h"

typedef struct {
	float roll, pitch, yaw;				/* 融合后的姿态输出 */
	float accel_roll, accel_pitch;		/* 加速度计算的角度 （中间量） */
} Attitude_t;

extern Attitude_t att1, att2;


void ICM_GetRollPitch(IMU_Data_t *imu_data, Attitude_t *att);
void Attitude_Update(IMU_Data_t *imu_data, Attitude_t *att, float dt);
void Attitude_CptYaw(const QMC_Data_t *qmc_data, Attitude_t *att);		/* Compute Yaw */

#endif
