#include "alg_attitude.h"
#include "math.h"

Attitude_t attitude = {0};

void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att)
{
    // 欧拉角旋转顺序：先Roll后Pitch，Pitch分母需补偿Roll的影响
    att->accel_roll = atan2f(imu->ay, imu->az);

    float az = imu->az;
    float ay = imu->ay;
    att->accel_pitch = atan2f(-imu->ax, sqrtf(az * az + ay * ay));
}

void Attitude_Update(const IcmData_t *imu, Attitude_t *att, float dt)
{
    float GyroX, GyroY;

    // 陀螺仪单位由dps转为rad/s
    GyroX = imu->gx * M_PI / 180.0f;
    GyroY = imu->gy * M_PI / 180.0f;

    att->roll = GYRO_TRUST * (att->roll + GyroX * dt) + (1.0f - GYRO_TRUST) * att->accel_roll;
    att->pitch = GYRO_TRUST * (att->pitch + GyroY * dt) + (1.0f - GYRO_TRUST) * att->accel_pitch;
}

void Attitude_CptYaw(const MagData_t *mag, Attitude_t *att)
{
    float sin_roll = sinf(att->roll);
    float cos_roll = cosf(att->roll);
    float sin_pitch = sinf(att->pitch);
    float cos_pitch = cosf(att->pitch);

    // 依次进行 Roll、Pitch 倾斜补偿：Mh = Ry(Pitch) · Rx(Roll) · M
    // 第一步：消除Roll
    float MY_h = cos_roll * mag->MY - sin_roll * mag->MZ;
    float MZ_ = sin_roll * mag->MY + cos_roll * mag->MZ;

    // 第二步：消除Pitch
    float MX_h = cos_pitch * mag->MX + sin_pitch * MZ_;

    // 利用水平磁场计算航向角
    att->yaw = atan2f(-MY_h, MX_h);
}
