/**
 * @file    app_imu_calibration.c
 * @brief   双 IMU Gyro Bias 上电静止校准实现。
 * 
 * 校准期间仅接受持续静止状态下的样本。
 * 若确认机体发生移动，则两颗 Required IMU 的累计窗口同时重新开始，
 * 保证最终 Bias 来自同一段连续静止时间。
 */

#include "app_imu_calibration.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

/*
 * Gyro Bias 校准条件。
 * 800 Hz 下累计 1600 个有效静止样本约对应 2s。
 */
#define GYRO_CAL_SAMPLE_COUNT 1600U         // 每颗 Required IMU 需要累计的静止样本数。
#define GYRO_CAL_MAX_ABS_DPS 3.0f           // 静止判断允许的最大各轴 Gyro 绝对值，deg/s。
#define GYRO_CAL_MAX_STDDEV_DPS 0.40f       // 校准窗口允许的最大 Gyro 标准差，deg/s。
#define GYRO_CAL_ACCEL_NORM_MIN_G 0.85f     // 静止判断允许的最小 Accel 模长，g。
#define GYRO_CAL_ACCEL_NORM_MAX_G 1.15f     // 静止判断允许的最大 Accel 模长，g。
#define GYRO_CAL_MOTION_CONFIRM_COUNT 4U    // 判定真实运动前要求的连续非静止样本数。

/**
 * @brief   单颗 IMU 的 Gyro Bias 校准累计状态。
 */
typedef struct
{
    uint32_t count;     // 当前已累计的有效静止样本数。
    float sum[3];       // 各轴 Gyro 样本和，用于计算均值。
    float sum_sq[3];    // 各轴 Gyro 平方和，用于计算方差。
    float bias[3];      // 最终计算得到的各轴 Gyro Bias，deg/s。
} GyroCalSensor_t;

static GyroCalSensor_t s_sensor[ICM_INSTANCE_MAX];  // 每颗 IMU 独立维护的校准累计状态。
static uint8_t s_required_mask;     // 本次启动中要求完成校准的 IMU 位掩码。
static bool s_ready;                // 所有 Required IMU 是否已经完成校准。

/*
 * 各 IMU 连续非静止样本计数。
 * 用于过滤偶发 Gyro/Accel Spike，避免单帧异常直接清空整个校准窗口。
 */
static uint8_t s_nonstationary_streak[ICM_INSTANCE_MAX];

/*
 * 清除所有 IMU 当前累计的校准窗口。
 * 
 * Bias 数组不在这里清零，因为本函数只用于重新采样；
 * 校准完成前旧 Bias 不会被 Apply 使用。
 */
static void ImuCalibration_ResetAccumulators(void)
{
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        s_sensor[i].count = 0U;
        memset(s_sensor[i].sum, 0, sizeof(s_sensor[i].sum));
        memset(s_sensor[i].sum_sq, 0, sizeof(s_sensor[i].sum_sq));

        s_nonstationary_streak[i] = 0U;
    }
}

/*
 * 判断单帧 IMU 数据是否满足静止采样条件。
 * 
 * 同时检查各轴 Gyro 绝对值和 Accel 三轴模长；
 * Accel 使用模长而非单轴值，因此允许机体以任意静止姿态放置。
 */
static bool ImuCalibration_SampleIsStationary(const IcmData_t *data)
{
    if(data == NULL)
        return false;

    if(fabsf(data->gx) > GYRO_CAL_MAX_ABS_DPS ||
       fabsf(data->gy) > GYRO_CAL_MAX_ABS_DPS ||
       fabsf(data->gz) > GYRO_CAL_MAX_ABS_DPS)
    {
        return false;
    }

    /* 使用平方模长比较，避免仅为静止判断额外执行 sqrtf()。 */
    float accel_norm_sq = data->ax * data->ax +
                          data->ay * data->ay +
                          data->az * data->az;

    const float norm_min_sq = GYRO_CAL_ACCEL_NORM_MIN_G * GYRO_CAL_ACCEL_NORM_MIN_G;
    const float norm_max_sq = GYRO_CAL_ACCEL_NORM_MAX_G * GYRO_CAL_ACCEL_NORM_MAX_G;

    return accel_norm_sq >= norm_min_sq && accel_norm_sq <= norm_max_sq;
}

/*
 * 将一帧静止 Gyro 数据累计到对应 IMU 的校准窗口。
 * 
 * 同时累计 x 与 x^2，使最终可以由
 * E[x^2] - E[x]^2 直接计算样本分布方差。
 */
static void ImuCalibration_Accumulate(IcmInstance_t instance, const IcmData_t *data)
{
    GyroCalSensor_t *sensor = &s_sensor[(uint8_t)instance];

    const float gyro[3] = {data->gx, data->gy, data->gz};
    
    if(sensor->count >= GYRO_CAL_SAMPLE_COUNT)
        return;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        sensor->sum[axis] += gyro[axis];
        sensor->sum_sq[axis] += gyro[axis] * gyro[axis];
    }

    sensor->count++;
}

/*
 * 检查所有 Required IMU 是否都已经收集足够样本。
 * 
 * required_mask == 0 表示没有任何成功初始化且需要参与校准的 IMU，
 * 此状态不能视为校准完成。
 */
static bool ImuCalibration_AllRequiredSamplesCollected(void)
{
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) != 0U &&
            s_sensor[i].count < GYRO_CAL_SAMPLE_COUNT)
        {
            return false;
        }
    }

    return s_required_mask != 0U;
}

/*
 * 根据完整静止采集窗口计算各 Required IMU 的 Gyro Bias。
 * 
 * Bias 取各轴样本均值；
 * 同时检查窗口标准差，防止表面满足单帧静止门限但整体仍存在振动。
 */
static bool ImuCalibration_CalculateBias(void)
{
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U)
            continue;

        const float count = (float)s_sensor[i].count;

        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            float mean = s_sensor[i].sum[axis] / count;
            float variance = s_sensor[i].sum_sq[axis] / count - mean * mean;

            /*
             * 浮点舍入可能使理论上为零的方差出现极小负值，
             * 此处钳制为零后再计算标准差。
             */
            if(variance < 0.0f)
                variance = 0.0f;
            
            if(sqrtf(variance) > GYRO_CAL_MAX_STDDEV_DPS)
                return false;

            s_sensor[i].bias[axis] = mean;
        }
    }

    return true;
}

void ImuCalibration_Init(uint8_t required_mask)
{
    memset(s_sensor, 0, sizeof(s_sensor));
    memset(s_nonstationary_streak, 0, sizeof(s_nonstationary_streak));

    /*
     * 只保留当前实现支持的 IMU 位，
     * 防止调用方传入无效高位污染状态机。
     */
    s_required_mask = required_mask & 
            (IMU_CAL_REQUIRED_IMU1 | IMU_CAL_REQUIRED_IMU2);
    s_ready = false;
}

bool ImuCalibration_Update(
    const IcmData_t *imu1,
    bool imu1_fresh,
    const IcmData_t *imu2,
    bool imu2_fresh)
{
    if(s_ready)
        return true;

    if(s_required_mask == 0U)
        return false;

    const IcmData_t *data[ICM_INSTANCE_MAX] = {imu1, imu2};

    const bool fresh[ICM_INSTANCE_MAX] = {imu1_fresh, imu2_fresh};

    bool any_required_fresh = false;

    /*
     * 先检查本轮所有 Required IMU 的新样本是否仍满足静止条件，
     * 通过后才真正累计样本。
     * 
     * 这样可以保证一旦确认机体发生移动，两颗 IMU 都从同一新的
     * 静止时间窗口重新开始，而不会拼接移动后前后的数据。
     */
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U || !fresh[i])
            continue;

        any_required_fresh = true;
        if(!ImuCalibration_SampleIsStationary(data[i]))
        {
            /*
             * 单帧异常可能只是 Sensor Spike，
             * 当前帧不参与 Bias 累计，但暂时保留此前静止样本。
             */
            if(s_nonstationary_streak[i] < GYRO_CAL_MOTION_CONFIRM_COUNT)
            {
                s_nonstationary_streak[i]++;
            }
            
            /*
             * 同一颗 IMU 连续多帧不满足静止条件后，
             * 才确认当前静止窗口已经被真实运动破坏。
             * 
             * 两颗 IMU 同时清空累计状态，
             * 保证最终 Bias 来自同一段连续静止区间。
             */
            if(s_nonstationary_streak[i] >= GYRO_CAL_MOTION_CONFIRM_COUNT)
            {
                ImuCalibration_ResetAccumulators();
            }
            return false;
        }

        // 当前样本重新满足静止条件，清除连续异常计数。
        s_nonstationary_streak[i] = 0U;
    }

    // 本周期没有任何 Required IMU 新样本时不推进校准。
    if(!any_required_fresh)
        return false;

    if((s_required_mask & IMU_CAL_REQUIRED_IMU1) != 0U && imu1_fresh)
    {
        ImuCalibration_Accumulate(ICM_INSTANCE_1, imu1);
    }

    if((s_required_mask & IMU_CAL_REQUIRED_IMU2) != 0U && imu2_fresh)
    {
        ImuCalibration_Accumulate(ICM_INSTANCE_2, imu2);
    }

    /*
     * 双 IMU 的采样频率或 DRDY 到达时刻可以不同，
     * 因此分别累计，直到所有 Required IMU 都达到目标样本数。
     */
    if(!ImuCalibration_AllRequiredSamplesCollected())
        return false;

    /*
     * 样本数量满足后再进行整体稳定性检查。
     * 若任一 Required IMU 的任一轴标准差超限，则整段窗口作废重采。
     */
    if(!ImuCalibration_CalculateBias())
    {
        ImuCalibration_ResetAccumulators();
        return false;
    }

    s_ready = true;

    IndicatorEvent_t evt = EVT_GYRO_CALIB_SUCCESS;
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
    return true;
}

void ImuCalibration_Apply(IcmInstance_t instance , IcmData_t *data)
{
    if(!s_ready || data == NULL || instance>= ICM_INSTANCE_MAX)
        return;

    const GyroCalSensor_t *sensor = &s_sensor[(uint8_t)instance];
    
    // 将本次启动估计出的静态 Gyro Bias 从原始测量中扣除。
    data->gx -= sensor->bias[0];
    data->gy -= sensor->bias[1];
    data->gz -= sensor->bias[2];
}

bool ImuCalibration_IsReady(void)
{
    return s_ready;
}
