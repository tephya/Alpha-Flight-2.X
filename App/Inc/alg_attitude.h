#ifndef __ALG_ATTITUDE_H
#define __ALG_ATTITUDE_H

#include "bsp_icm42688.h"
#include "bsp_qmc5883.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define GYRO_TRUST 0.998f

typedef struct
{
    float roll, pitch, yaw;         // 融合后的姿态输出，单位：rad
    float accel_roll, accel_pitch;  // 加速度计算的角度(中间量)，单位：rad
} Attitude_t;

extern Attitude_t attitude;

void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att);
void Attitude_Update(const IcmData_t *imu, Attitude_t *att, float dt);
void Attitude_CptYaw(const MagData_t *mag, Attitude_t *att);

#endif
