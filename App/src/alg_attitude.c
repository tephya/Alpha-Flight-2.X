/**
 * @file    alg_attitude.c
 * @brief   Roll/Pitch互补滤波及Mag倾斜补偿Yaw的实现。
 */

#include "alg_attitude.h"
#include "math.h"

/*
 * Disarmed时，机体应当基本静止，可以较快利用重力方向修正Gyro积分漂移。
 * 0.65s约等于原来在当前控制频率下GYRO_TRUST = 0.998的实际时间常数。 
 */
#define ATTITUDE_ACCEL_TAU_DISARMED_S 0.65f

/*
 * Armed时可能存在水平线加速度，此时Accel观测不再只包含重力。
 * 增大时间常数，使姿态主要跟随Gyro，同时仍保留缓慢的长期漂移修正。
 */
#define ATTITUDE_ACCEL_TAU_ARMED_S 8.0f


/* FlightControl使用的单一全局姿态状态。 */
Attitude_t attitude = {0};

void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att)
{
    /**
     * 静止或低动态情况下，Accel测得的主要是重力方向。
     * 
     * Roll由Y/Z轴的重力投影得到；使用atan2f保留象限信息。
     * 并避免单纯ay/az在az接近0时产生除零问题。
     */
    att->accel_roll = atan2f(imu->ay, imu->az);

    /**
     * Pitch使用Y/Z平面重力投影的模作为分母。
     * 这样可降低Roll变化对Pitch观测值的直接影响；
     * 
     * pitch = atan2(-ax, sqrtf(ay² + az²))
     */
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

    /* ICM驱动输出单位为dps，积分前转换为rad/s */
    const float gyro_x_rad_s = imu->gx * M_PI / 180.0f;
    const float gyro_y_rad_s = imu->gy * M_PI / 180.0f;

    const float correction_tau_s = use_weak_accel_correction ?
                                    ATTITUDE_ACCEL_TAU_ARMED_S :
                                    ATTITUDE_ACCEL_TAU_DISARMED_S;

    /*
     * 根据实际dt计算互补滤波系数，使滤波时间常数不再依赖控制频率。
     * 
     * alpha接近1时更信任Gyro；
     * (1 - alpha)决定本周期Accel观测的校正权重。
     */
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

    /**
     * Mag随飞行器一起倾斜，不能直接使用MX/MY计算Yaw。
     * 这里按照当前姿态依次消除Roll和Pitch：
     * 
     * Mag_horizontal = Ry(Pitch) · Rx(Roll) · Mag_raw
     */

    /* 第一步：消除Roll，得到Roll补偿后的Y/Z分量 */
    float MY_h = cos_roll * mag->MY - sin_roll * mag->MZ;
    float MZ_ = sin_roll * mag->MY + cos_roll * mag->MZ;

    /* 第二步：利用Roll补偿后的Z分量消除Pitch，得到补偿后的X分量 */
    float MX_h = cos_pitch * mag->MX + sin_pitch * MZ_;

    /**
     * 使用水平磁场分量计算Yaw。
     * Y轴取负与当前项目的NED轴向和Yaw正方向约定一致。
     */
    att->yaw = atan2f(-MY_h, MX_h);
}
