#include "app_level_trim.h"
#include "app_imu_calibration.h"
#include "app_shared_types.h"
#include "bsp_config_flash.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

#define LEVEL_TRIM_CONFIG_RECORD_TYPE 0x0002U
#define LEVEL_TRIM_CONFIG_VERSION 1U

#define LEVEL_TRIM_SAMPLE_COUNT 1600U   /* 800Hz下约2s */
#define LEVEL_TRIM_CAPTURE_TIMEOUT_MS 15000U
#define LEVEL_TRIM_FILTER_SETTLE_MS 1000U

#define LEVEL_TRIM_RC_CHANNEL_SE 6U /* CRSF CH7 -> channels[6] */
#define LEVEL_TRIM_RC_CHANNEL_ARM 4U /* SD */
#define LEVEL_TRIM_RC_CHANNEL_THROTTLE  2U
#define LEVEL_TRIM_SE_PRESSED_MIN   1500U
#define LEVEL_TRIM_SE_RELEASED_MAX 900U
#define LEVEL_TRIM_ARM_OFF_MAX  900U
#define LEVEL_TRIM_THROTTLE_LOW_MAX 180U
#define LEVEL_TRIM_TRIGGER_HOLD_MS 3000U

#define LEVEL_TRIM_MAX_GYRO_ABS_DPS 3.0f
#define LEVEL_TRIM_ACCEL_NORM_MIN_G 0.85f
#define LEVEL_TRIM_ACCEL_NORM_MAX_G 1.15f
#define LEVEL_TRIM_DEG_TO_RAD 0.01745329252f
#define LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD (10.0f * LEVEL_TRIM_DEG_TO_RAD)
#define LEVEL_TRIM_MAX_STDDEV_RAD   (0.75f * LEVEL_TRIM_DEG_TO_RAD)
#define LEVEL_TRIM_MAX_STORED_OFFSET_RAD (10.0f * LEVEL_TRIM_DEG_TO_RAD)

typedef struct
{
    float roll_offset_rad[ICM_INSTANCE_MAX];
    float pitch_offset_rad[ICM_INSTANCE_MAX];
    uint8_t valid_mask;
    uint8_t reserved[3];
} LevelTrimConfig_t;

typedef char LevelTrimPayloadMustFitConfigFlash[
    (sizeof(LevelTrimConfig_t) <= CONFIG_FLASH_PAYLOAD_MAX) ? 1 : -1
];

typedef struct
{
    uint32_t count;
    float roll_sum;
    float roll_sum_sq;
    float pitch_sum;
    float pitch_sum_sq;
} LevelTrimAccumulator_t;

typedef enum
{
    LEVEL_TRIM_STATE_IDLE = 0,
    LEVEL_TRIM_STATE_COLLECTING,
    LEVEL_TRIM_STATE_FILTER_SETTLING,
} LevelTrimState_t;

static LevelTrimConfig_t s_config;
static LevelTrimAccumulator_t s_acc[ICM_INSTANCE_MAX];
static uint8_t s_required_mask;
static LevelTrimState_t s_state = LEVEL_TRIM_STATE_IDLE;

static bool s_trigger_holding;
static bool s_trigger_latched;
static uint32_t s_trigger_start_tick;
static uint32_t s_capture_start_tick;
static uint32_t s_settle_start_tick;

static void LevelTrim_PostEvent(IndicatorEvent_t evt)
{
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}

static void LevelTrim_ResetAccumulators(void)
{
    memset(s_acc, 0, sizeof(s_acc));
}

static bool LevelTrim_ValueInRange(float value, float abs_limit)
{
    /* NaN是唯一一个与自身不相等的float；后续范围比较同时排除Inf。 */
    return (value == value && value >= -abs_limit && value <= abs_limit);
}

static bool LevelTrim_ConfigValid(const LevelTrimConfig_t *config)
{
    const uint8_t all_imu_mask = (uint8_t)((1U << (uint8_t)ICM_INSTANCE_MAX) - 1U);
    
    if(config == NULL || config->valid_mask == 0U || (config->valid_mask & (uint8_t)~all_imu_mask) != 0U)
    {
        return false;
    }

    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((config->valid_mask & (1U << i))== 0U)
            continue;

        if(!LevelTrim_ValueInRange(config->roll_offset_rad[i],
                                    LEVEL_TRIM_MAX_STORED_OFFSET_RAD) ||
            !LevelTrim_ValueInRange(config->pitch_offset_rad[i],
                                    LEVEL_TRIM_MAX_STORED_OFFSET_RAD))
        {
            return false;
        }
    }

    return true;
}

static void LevelTrim_Fail(void)
{
    s_state = LEVEL_TRIM_STATE_IDLE;
    LevelTrim_ResetAccumulators();
    LevelTrim_PostEvent(EVT_LEVEL_TRIM_FAILED);
}

static void LevelTrim_Start(void)
{
    if(s_required_mask == 0U)
    {
        LevelTrim_PostEvent(EVT_LEVEL_TRIM_FAILED);
        return;
    }

    LevelTrim_ResetAccumulators();
    s_capture_start_tick = osKernelGetTickCount();
    s_state = LEVEL_TRIM_STATE_COLLECTING;
    LevelTrim_PostEvent(EVT_LEVEL_TRIM_STARTED);
}

static bool LevelTrim_CalculateAccelAngles(const IcmData_t *imu, float *roll_rad, float *pitch_rad)
{
    if(imu == NULL || roll_rad == NULL || pitch_rad == NULL)
        return false;

    if(fabsf(imu->gx) > LEVEL_TRIM_MAX_GYRO_ABS_DPS ||
        fabsf(imu->gy) > LEVEL_TRIM_MAX_GYRO_ABS_DPS ||
        fabsf(imu->gz) > LEVEL_TRIM_MAX_GYRO_ABS_DPS)
    {
        return false;
    }

    const float accel_norm_sq = imu->ax * imu->ax +
                                imu->ay * imu->ay +
                                imu->az * imu->az;

    const float norm_min_sq = LEVEL_TRIM_ACCEL_NORM_MIN_G * LEVEL_TRIM_ACCEL_NORM_MIN_G;
    const float norm_max_sq = LEVEL_TRIM_ACCEL_NORM_MAX_G * LEVEL_TRIM_ACCEL_NORM_MAX_G;

    if(accel_norm_sq < norm_min_sq || accel_norm_sq > norm_max_sq)
        return false;

    *roll_rad = atan2f(imu->ay, imu->az);
    *pitch_rad = atan2f(-imu->ax, sqrtf(imu->az * imu->az + imu->ay * imu->ay));

    return fabsf(*roll_rad) <= LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD &&
           fabsf(*pitch_rad) <= LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD;
}

static void LevelTrim_Accumulate(IcmInstance_t instance, float roll_rad, float pitch_rad)
{
    LevelTrimAccumulator_t *acc = &s_acc[(uint8_t)instance];

    if(acc->count >= LEVEL_TRIM_SAMPLE_COUNT)
        return;

    acc->roll_sum += roll_rad;
    acc->roll_sum_sq += roll_rad * roll_rad;
    acc->pitch_sum += pitch_rad;
    acc->pitch_sum_sq += pitch_rad * pitch_rad;
    acc->count++;
}

static bool LevelTrim_AllSamplesCollected(void)
{
    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) != 0U &&
            s_acc[i].count < LEVEL_TRIM_SAMPLE_COUNT)
        {
            return false;
        }
    }

    return s_required_mask != 0U;
}

static bool LevelTrim_BuildCandidate(LevelTrimConfig_t *candidate)
{
    if(candidate == NULL)
        return false;

    *candidate = s_config;  // 保留本次未参与校准的IMU旧结果

    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U)
            continue;

        const float count = (float)s_acc[i].count;
        const float roll_mean = s_acc[i].roll_sum / count;
        const float pitch_mean = s_acc[i].pitch_sum / count;

        float roll_variance = s_acc[i].roll_sum_sq / count - roll_mean * roll_mean;
        float pitch_variance = s_acc[i].pitch_sum_sq / count - pitch_mean * pitch_mean;
        
        if(roll_variance < 0.0f)
            roll_variance = 0.0f;
        if(pitch_variance < 0.0f)
            pitch_variance = 0.0f;

        if(sqrtf(roll_variance) > LEVEL_TRIM_MAX_STDDEV_RAD ||
            sqrtf(pitch_variance) > LEVEL_TRIM_MAX_STDDEV_RAD)
        {
            return false;
        }

        candidate->roll_offset_rad[i] = roll_mean;
        candidate->pitch_offset_rad[i] = pitch_mean;
        candidate->valid_mask |= (uint8_t)(1U << i);
    }

    return LevelTrim_ConfigValid(candidate);
}

void LevelTrim_Init(uint8_t required_mask)
{
    const uint8_t all_imu_mask = (uint8_t)((1U << (uint8_t)ICM_INSTANCE_MAX) - 1U);

    s_required_mask = required_mask & all_imu_mask;
    s_state = LEVEL_TRIM_STATE_IDLE;
    s_trigger_holding = false;
    s_trigger_latched = false;
    s_trigger_start_tick = 0U;
    s_capture_start_tick = 0U;
    s_settle_start_tick = 0U;
    LevelTrim_ResetAccumulators();
    memset(&s_config, 0, sizeof(s_config));

    LevelTrimConfig_t stored;
    if(BSP_ConfigFlash_LoadLatest(LEVEL_TRIM_CONFIG_RECORD_TYPE,
                                    LEVEL_TRIM_CONFIG_VERSION,
                                    &stored,
                                    sizeof(stored),
                                    NULL) == CONFIG_FLASH_OK &&
        LevelTrim_ConfigValid(&stored) &&
        s_required_mask != 0U &&
        (stored.valid_mask & s_required_mask) == s_required_mask)
    {
        /* 当前可用IMU必须全部有Trim才整体启用，避免只补偿其中一颗，
         * 主备切换时产生姿态基准跃迁。 */
        s_config = stored;
    }
}

void LevelTrim_HandleRc(const RCChannelData_t *rc)
{
    if(rc == NULL)
        return;

    const bool se_released = rc->channels[LEVEL_TRIM_RC_CHANNEL_SE] <= LEVEL_TRIM_SE_RELEASED_MAX;

    if(se_released)
    {
        s_trigger_holding = false;
        s_trigger_latched = false;
        return;
    }

    if(s_state != LEVEL_TRIM_STATE_IDLE || s_trigger_latched)
        return;

    const bool eligible =
        rc->link_ok &&
        g_arm_state == ARM_STATE_DISARMED &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_ARM] <= LEVEL_TRIM_ARM_OFF_MAX &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_THROTTLE] <= LEVEL_TRIM_THROTTLE_LOW_MAX &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_SE] >= LEVEL_TRIM_SE_PRESSED_MIN &&
        ImuCalibration_IsReady();
    
    if(!eligible)
    {
        s_trigger_holding = false;
        return;
    }

    const uint32_t now = osKernelGetTickCount();

    if(!s_trigger_holding)
    {
        s_trigger_holding = true;
        s_trigger_start_tick = now;
        return;
    }

    if((now - s_trigger_start_tick) >= LEVEL_TRIM_TRIGGER_HOLD_MS)
    {
        s_trigger_holding = false;
        s_trigger_latched = true;   // 必须先松开SE，才允许下一次触发
        LevelTrim_Start();
    }
}

void LevelTrim_UpdateSamples(const IcmData_t *imu1, bool imu1_fresh,
                            const IcmData_t *imu2, bool imu2_fresh)
{
    const uint32_t now = osKernelGetTickCount();

    if(s_state == LEVEL_TRIM_STATE_FILTER_SETTLING)
    {
        if((now - s_settle_start_tick) >= LEVEL_TRIM_FILTER_SETTLE_MS)
        {
            s_state = LEVEL_TRIM_STATE_IDLE;
            LevelTrim_PostEvent(EVT_LEVEL_TRIM_SUCCESS);
        }
        return;
    }

    if(s_state != LEVEL_TRIM_STATE_COLLECTING)
        return;
    
    if(g_arm_state != ARM_STATE_DISARMED || 
        (now - s_capture_start_tick) >= LEVEL_TRIM_CAPTURE_TIMEOUT_MS)
    {
        LevelTrim_Fail();
        return;
    }

    const IcmData_t *imu[ICM_INSTANCE_MAX] = {imu1, imu2};
    const bool fresh[ICM_INSTANCE_MAX] = {imu1_fresh, imu2_fresh};

    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U || !fresh[i])
            continue;

        float roll_rad;
        float pitch_rad;

        if(!LevelTrim_CalculateAccelAngles(imu[i], &roll_rad, &pitch_rad))
        {
            /* 任意一颗参与校准的IMU检测到移动，全部重新累计，保证同一静止窗口 */
            LevelTrim_ResetAccumulators();
            return;
        }

        LevelTrim_Accumulate((IcmInstance_t)i, roll_rad, pitch_rad);
    }

    if(!LevelTrim_AllSamplesCollected())
        return;

    LevelTrimConfig_t candidate;
    if(!LevelTrim_BuildCandidate(&candidate))
    {
        LevelTrim_Fail();
        return;
    }

    if(BSP_ConfigFlash_Append(LEVEL_TRIM_CONFIG_RECORD_TYPE,
                                LEVEL_TRIM_CONFIG_VERSION,
                                &candidate,
                                sizeof(candidate),
                                NULL) != CONFIG_FLASH_OK)
    {
        LevelTrim_Fail();
        return;
    }

    s_config = candidate;
    LevelTrim_ResetAccumulators();

    /* 新Trim生效后给互补滤波器1s收敛时间，期间LevelTrim_IsActive仍为true。 */
    s_settle_start_tick = now;
    s_state = LEVEL_TRIM_STATE_FILTER_SETTLING;
}

void LevelTrim_Apply(IcmInstance_t instance, float *accel_roll_rad, float *accel_pitch_rad)
{
    const uint8_t index = (uint8_t)instance;

    if(index >= (uint8_t)ICM_INSTANCE_MAX || 
        accel_roll_rad == NULL || accel_pitch_rad == NULL ||
        (s_config.valid_mask & (1U << index)) == 0U)
    {
        return;
    }

    *accel_roll_rad -= s_config.roll_offset_rad[index];
    *accel_pitch_rad -= s_config.pitch_offset_rad[index];
}

bool LevelTrim_IsActive(void)
{
    return s_state != LEVEL_TRIM_STATE_IDLE;
}

bool LevelTrim_GetOffsets(IcmInstance_t instance,
                          float *roll_offset_rad,
                          float *pitch_offset_rad)
{
    const uint8_t index = (uint8_t)instance;

    if(index >= (uint8_t)ICM_INSTANCE_MAX ||
        roll_offset_rad == NULL ||
        pitch_offset_rad == NULL ||
        (s_config.valid_mask & (1U << index)) == 0U)
    {
        return false;
    }

    *roll_offset_rad = s_config.roll_offset_rad[index];
    *pitch_offset_rad = s_config.pitch_offset_rad[index];
    return true;
}

bool LevelTrim_IsReady(void)
{
    return s_required_mask != 0U &&
           (s_config.valid_mask & s_required_mask) == s_required_mask;
}

