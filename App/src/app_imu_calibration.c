#include "app_imu_calibration.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

// 800Hz下1600个样本约为2s。只接受静止样本，移动时整段重新计数。
#define GYRO_CAL_SAMPLE_COUNT 1600U
#define GYRO_CAL_MAX_ABS_DPS 3.0f   // Gyro Calibration Maximum Absolute Degree Per Second
#define GYRO_CAL_MAX_STDDEV_DPS 0.40f   // standard deviation，标准差
#define GYRO_CAL_ACCEL_NORM_MIN_G 0.85f // Norm：模长，加速度模长最低阈值
#define GYRO_CAL_ACCEL_NORM_MAZ_G 1.15f
#define GYRO_CAL_MOTION_CONFIRM_COUNT 4U

typedef struct
{
    uint32_t count;
    float sum[3];
    float sum_sq[3];
    float bias[3];
} GyroCalSensor_t;

static GyroCalSensor_t s_sensor[ICM_INSTANCE_MAX];
static uint8_t s_required_mask;
static bool s_ready;
static uint8_t s_nonstationary_streak[ICM_INSTANCE_MAX];

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

    float accel_norm_sq = data->ax * data->ax +
                          data->ay * data->ay +
                          data->az * data->az;
    const float norm_min_sq = GYRO_CAL_ACCEL_NORM_MIN_G * GYRO_CAL_ACCEL_NORM_MIN_G;
    const float norm_max_sq = GYRO_CAL_ACCEL_NORM_MAZ_G * GYRO_CAL_ACCEL_NORM_MAZ_G;

    return accel_norm_sq >= norm_min_sq && accel_norm_sq <= norm_max_sq;
}

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
    // mask为0表示没有任何成功初始化、可参与校准的IMU
    return s_required_mask != 0U;
}

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

            // 浮点舍入可能令本应为零的方差略小于零
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

    s_required_mask = required_mask & (IMU_CAL_REQUIRED_IMU1 | IMU_CAL_REQUIRED_IMU2);
    s_ready = false;
}

bool ImuCalibration_Update(const IcmData_t *imu1, bool imu1_fresh, const IcmData_t *imu2, bool imu2_fresh)
{
    if(s_ready)
        return true;

    if(s_required_mask == 0U)
        return false;

    const IcmData_t *data[ICM_INSTANCE_MAX] = {imu1, imu2};
    const bool fresh[ICM_INSTANCE_MAX] = {imu1_fresh, imu2_fresh};
    bool any_required_fresh = false;

    /* 任一待校准IMU的新样本显示机体在移动，旧丢弃整段样本。
     * 两颗IMU必须来自同一次静止区间，不能拼接移动前后的数据。 */
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U || !fresh[i])
            continue;

        any_required_fresh = true;
        if(!ImuCalibration_SampleIsStationary(data[i]))
        {
            /* 单帧Gyro/Accel尖刺不代表机体真的发生移动
             * 本帧不参与bias累计，但保留此前已经收集的静止样本 */
            if(s_nonstationary_streak[i] < GYRO_CAL_MOTION_CONFIRM_COUNT)
            {
                s_nonstationary_streak[i]++;
            }
            /* 同一颗IMU连续多帧越界时，才认为静止条件已经被破坏，
             * 此时两颗IMU同一重新开始，保证他们仍属于同一静止窗口 */
            if(s_nonstationary_streak[i] >= GYRO_CAL_MOTION_CONFIRM_COUNT)
            {
                ImuCalibration_ResetAccumulators();
            }
            return false;
        }

        s_nonstationary_streak[i] = 0U;
    }

    if(!any_required_fresh)
        return false;

    if((s_required_mask & IMU_CAL_REQUIRED_IMU1) != 0U && imu1_fresh)
        ImuCalibration_Accumulate(ICM_INSTANCE_1, imu1);

    if((s_required_mask & IMU_CAL_REQUIRED_IMU2) != 0U && imu2_fresh)
        ImuCalibration_Accumulate(ICM_INSTANCE_2, imu2);

    if(!ImuCalibration_AllRequiredSamplesCollected())
        return false;

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
    data->gx -= sensor->bias[0];
    data->gy -= sensor->bias[1];
    data->gz -= sensor->bias[2];
}

bool ImuCalibration_IsReady(void)
{
    return s_ready;
}
