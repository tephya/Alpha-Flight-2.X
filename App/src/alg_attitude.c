/**
 * @file    alg_attitude.c
 * @brief   姿态角计算实现。
 * 
 * 实现 Roll/Pitch 互补滤波及基于 Mag 的倾斜补偿 Yaw 计算。
 */

#include "alg_attitude.h"
#include "math.h"

/**
 * Disarmed 状态下的 Accel 校正时间常数。
 * 机体静止时可较快利用重力观测修正 Gyro 积分漂移。
 */
#define ATTITUDE_ACCEL_TAU_DISARMED_S 0.65f

/**
 * Armed 状态下的 Accel 校正时间常数。
 * 较弱的 Accel 校正可降低线加速度对姿态估计的干扰。
 */
#define ATTITUDE_ACCEL_TAU_ARMED_S 8.0f


/* FlightControl 使用的全局姿态状态。 */
Attitude_t attitude = {0};

void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att)
{
    // 根据重力在 Y/Z 轴的投影计算 Roll。
    // atan2f 可保留象限信息，并避免 az 接近零时的除零问题。
    att->accel_roll = atan2f(imu->ay, imu->az);

    // 使用 Y/Z 平面重力投影计算 Pitch：
    // pitch = atan2(-ax, sqrtf(ay^2 + az^2))
    float az = imu->az;
    float ay = imu->ay;
    att->accel_pitch = atan2f(-imu->ax, sqrtf(az * az + ay * ay));
}

void Attitude_Update(const IcmData_t *imu,
                     Attitude_t *att,
                     float dt,
                     bool use_weak_accel_correction)
{
    if((imu == NULL) || (att == NULL) || (dt <= 0.0f))
    {
        return;
    }

    // ICM驱动输出单位为dps，积分前转换为 rad/s。
    const float gyro_x_rad_s = imu->gx * M_PI / 180.0f;
    const float gyro_y_rad_s = imu->gy * M_PI / 180.0f;

    const float correction_tau_s = use_weak_accel_correction ?
                                    ATTITUDE_ACCEL_TAU_ARMED_S :
                                    ATTITUDE_ACCEL_TAU_DISARMED_S;

    // 根据实际 dt 计算滤波系数，使时间常数不依赖控制频率。
    // alpha 越接近 1，姿态估计越依赖 Gyro 积分结果。
    const float alpha = expf(-dt / correction_tau_s);

    const float roll_gyro = att->roll + gyro_x_rad_s * dt;
    const float pitch_gyro = att->pitch + gyro_y_rad_s * dt;

    att->roll = alpha * roll_gyro + (1 - alpha) * att->accel_roll;
    att->pitch = alpha * pitch_gyro + (1 - alpha) * att->accel_pitch;
}

void Attitude_CptYaw(const MagData_t *mag, Attitude_t *att)
{
    float sin_roll = sinf(att->roll);
    float cos_roll = cosf(att->roll);
    float sin_pitch = sinf(att->pitch);
    float cos_pitch = cosf(att->pitch);

    /*
     * 将倾斜状态下的磁场向量补偿至水平面，
     * Mag_horizontal = Ry(Pitch) · Rx(Roll) · Mag_raw
     */

    // 消除Roll，得到Roll补偿后的Y/Z分量。
    float MY_h = cos_roll * mag->MY - sin_roll * mag->MZ;
    float MZ_ = sin_roll * mag->MY + cos_roll * mag->MZ;

    // 使用 Roll 补偿后的 Z 分量进一步消除 Pitch。
    float MX_h = cos_pitch * mag->MX + sin_pitch * MZ_;

    // Y 分量取负由当前 NED 坐标系及 Yaw 正方向约定决定。
    att->yaw = atan2f(-MY_h, MX_h);
}
