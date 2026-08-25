/**
 * @file    app_level_trim.c
 * @brief   Level Trim 静止采样，Flash 持久化及姿态观测补偿实现。
 */

#include "app_level_trim.h"
#include "app_imu_calibration.h"
#include "app_shared_types.h"
#include "app_mag_calibration.h"
#include "bsp_config_flash.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

/* Config Flash 中 Level Trim Record 的类型和版本。*/
#define LEVEL_TRIM_CONFIG_RECORD_TYPE 0x0002U
#define LEVEL_TRIM_CONFIG_VERSION 1U

#define LEVEL_TRIM_SAMPLE_COUNT 1600U           // 每颗 Required IMU 采样数量，约 2 s @ 800 Hz。
#define LEVEL_TRIM_CAPTURE_TIMEOUT_MS 15000U    // 单次 Level Trim 最大允许采样时间，ms。
#define LEVEL_TRIM_FILTER_SETTLE_MS 1000U       // 新 Trim 生效后姿态滤波器稳定等待时间，ms。

/* RC 触发条件。 */
#define LEVEL_TRIM_RC_CHANNEL_SE 6U             // SE：CRSF CH7，对应 channel[6]。
#define LEVEL_TRIM_RC_CHANNEL_ARM 4U            // ARM 开关通道。
#define LEVEL_TRIM_RC_CHANNEL_THROTTLE  2U      // Throttle 通道。
#define LEVEL_TRIM_SE_PRESSED_MIN 1500U         // SE 判定为按下的最小通道值。
#define LEVEL_TRIM_SE_RELEASED_MAX 900U         // SE 判定为释放的最大通道值。
#define LEVEL_TRIM_ARM_OFF_MAX 900U             // ARM 开关判定为关闭的最大通道值。
#define LEVEL_TRIM_THROTTLE_LOW_MAX 200U        // 允许触发校准的最大低油门值。
#define LEVEL_TRIM_TRIGGER_HOLD_MS 3000U        // SE 连续按住触发时间，ms。

/* Level Trim 静止采样及结果有效性门限。 */
#define LEVEL_TRIM_MAX_GYRO_ABS_DPS 3.0f        // 静止判断允许的最大各轴 Gyro 绝对值，deg/s。
#define LEVEL_TRIM_ACCEL_NORM_MIN_G 0.85f       // 静止判断允许的最小 Accel 模长，g。
#define LEVEL_TRIM_ACCEL_NORM_MAX_G 1.15f       // 静止判断允许的最大 Accel 模长，g。

#define LEVEL_TRIM_DEG_TO_RAD 0.01745329252f
#define LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD \
    (10.0f * LEVEL_TRIM_DEG_TO_RAD)     // 允许采集的最大 Roll/Pitch 静态倾角，rad。
#define LEVEL_TRIM_MAX_STDDEV_RAD \
    (0.75f * LEVEL_TRIM_DEG_TO_RAD)     // 校准窗口允许的最大姿态角标准差，rad。
#define LEVEL_TRIM_MAX_STORED_OFFSET_RAD \
    (10.0f * LEVEL_TRIM_DEG_TO_RAD)     // Flash 中允许保存的最大 Trim 绝对值，rad。

/**
 * @brief   Level Trim 持久化配置。
 */
typedef struct
{
    float roll_offset_rad[ICM_INSTANCE_MAX];    /**< 各 IMU Roll Trim，rad。 */
    float pitch_offset_rad[ICM_INSTANCE_MAX];   /**< 各 IMU Pitch Trim，rad。 */
    uint8_t valid_mask;                         /**< 已存在有效 Trim 的 IMU 位掩码。 */
    uint8_t reserved[3];                        /**< 保留字段，用于结构对齐和后续扩展。 */
} LevelTrimConfig_t;

/* 编译期检查：Level Trim Payload 必须能装入 Config Flash Record。 */
typedef char LevelTrimPayloadMustFitConfigFlash[
    (sizeof(LevelTrimConfig_t) <= CONFIG_FLASH_PAYLOAD_MAX) ? 1 : -1
];

/**
 * @brief   单颗 IMU 的 Level Trim 采样累计状态。
 */
typedef struct
{
    uint32_t count;         /**< 当前有效样本数。 */
    float roll_sum;         /**< Roll 样本和。 */
    float roll_sum_sq;      /**< Roll 平方和，用于计算方差。 */
    float pitch_sum;        /**< Pitch 样本和。 */
    float pitch_sum_sq;     /**< Pitch 平方和，用于计算方差。 */
} LevelTrimAccumulator_t;

/**
 * @brief   Level Trim 状态机。
 */
typedef enum
{
    LEVEL_TRIM_STATE_IDLE = 0,        /**< 空闲，可等待新的 RC 触发。 */
    LEVEL_TRIM_STATE_COLLECTING,      /**< 正在采集静止姿态样本。 */
    LEVEL_TRIM_STATE_FILTER_SETTLING, /**< 新 Trim 已生效，等待姿态滤波重新稳定。 */
} LevelTrimState_t;

static LevelTrimConfig_t s_config;
static LevelTrimAccumulator_t s_acc[ICM_INSTANCE_MAX];

static uint8_t s_required_mask;
static LevelTrimState_t s_state = LEVEL_TRIM_STATE_IDLE;

/* RC 长按触发状态。 */
static bool s_trigger_holding;
static bool s_trigger_latched;
static uint32_t s_trigger_start_tick;

/* 校准状态时间基准。 */
static uint32_t s_capture_start_tick;
static uint32_t s_settle_start_tick;

/* 非阻塞向 Indicator Task 发布 Level Trim 状态事件。 */
static void LevelTrim_PostEvent(IndicatorEvent_t evt)
{
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}

/* 清除当前所有 IMU 的 Level Trim 采样累计窗口。 */
static void LevelTrim_ResetAccumulators(void)
{
    memset(s_acc, 0, sizeof(s_acc));
}

/*
 * 检查浮点值是否处于允许的有限范围。
 * value == value 排除 NaN，范围比较同时可排除 Inf。
 */
static bool LevelTrim_ValueInRange(float value, float abs_limit)
{
    return (value == value && 
            value >= -abs_limit && 
            value <= abs_limit);
}

/*
 * 检查一份 Level Trim 配置是否合法。
 *
 * 至少需要一颗 IMU 的有效标志；
 * valid_mask 不允许包含超出当前 IMU 数量的位；
 * 所有声明有效的 Offset 必须处于允许范围。
 */
static bool LevelTrim_ConfigValid(const LevelTrimConfig_t *config)
{
    const uint8_t all_imu_mask = 
        (uint8_t)((1U << (uint8_t)ICM_INSTANCE_MAX) - 1U);
    
    if(config == NULL || 
        config->valid_mask == 0U || 
        (config->valid_mask & (uint8_t)~all_imu_mask) != 0U)
    {
        return false;
    }

    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((config->valid_mask & (1U << i))== 0U)
            continue;

        if(!LevelTrim_ValueInRange(
                config->roll_offset_rad[i],
                LEVEL_TRIM_MAX_STORED_OFFSET_RAD) ||
            !LevelTrim_ValueInRange(
                config->pitch_offset_rad[i],
                LEVEL_TRIM_MAX_STORED_OFFSET_RAD))
        {
            return false;
        }
    }

    return true;
}

/* 终止当前校准并回到 Idle，同时发布失败提示。 */
static void LevelTrim_Fail(void)
{
    s_state = LEVEL_TRIM_STATE_IDLE;
    LevelTrim_ResetAccumulators();
    LevelTrim_PostEvent(EVT_LEVEL_TRIM_FAILED);
}

/* 开始一次新的 Level Trim 静止采样。 */
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

/*
 * 检查单帧 IMU 是否满足 Level Trim 静止采样条件，
 * 并根据重力方向计算 Accel Roll/Pitch Observation。
 */
static bool LevelTrim_CalculateAccelAngles(
    const IcmData_t *imu,
    float *roll_rad,
    float *pitch_rad)
{
    if(imu == NULL || roll_rad == NULL || pitch_rad == NULL)
        return false;

    // 任一轴 Gyro 过大均认为机体仍在运动。
    if(fabsf(imu->gx) > LEVEL_TRIM_MAX_GYRO_ABS_DPS ||
        fabsf(imu->gy) > LEVEL_TRIM_MAX_GYRO_ABS_DPS ||
        fabsf(imu->gz) > LEVEL_TRIM_MAX_GYRO_ABS_DPS)
    {
        return false;
    }

    /*
     * 使用 Accel 平方模长判断是否接近 1g，
     * 避免仅为静止时判断额外执行 sqrtf()。
     */
    const float accel_norm_sq = imu->ax * imu->ax +
                                imu->ay * imu->ay +
                                imu->az * imu->az;

    const float norm_min_sq = LEVEL_TRIM_ACCEL_NORM_MIN_G * LEVEL_TRIM_ACCEL_NORM_MIN_G;
    const float norm_max_sq = LEVEL_TRIM_ACCEL_NORM_MAX_G * LEVEL_TRIM_ACCEL_NORM_MAX_G;

    if(accel_norm_sq < norm_min_sq || accel_norm_sq > norm_max_sq)
        return false;

    *roll_rad = atan2f(imu->ay, imu->az);
    *pitch_rad = atan2f(
        -imu->ax, 
        sqrtf(imu->az * imu->az + imu->ay * imu->ay));

    /*
     * Level Trim 只允许校正小范围机械安装偏差。
     * 若机体本身明显倾斜，则拒绝作为“水平基准”采样。
     */
    return fabsf(*roll_rad) <= LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD &&
           fabsf(*pitch_rad) <= LEVEL_TRIM_MAX_CAPTURE_ANGLE_RAD;
}

/* 累计单颗 IMU 的 Roll/Pitch 样本及平方和。 */
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

/* 检查所有 Required IMU 是否都已经收集完整采样窗口。 */
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

/*
 * 根据当前采样窗口生成新的 Level Trim Candidate。
 * 
 * 本次未参与校准的 IMU 保留原有 Trim：
 * Required IMU 使用当前窗口均值更新，并通过标准差检查确认稳定性。
 */
static bool LevelTrim_BuildCandidate(LevelTrimConfig_t *candidate)
{
    if(candidate == NULL)
        return false;

    // 保留本次未参与校准的 IMU 旧结果
    *candidate = s_config;  

    for (uint8_t i = 0U; i < (uint8_t)ICM_INSTANCE_MAX; i++)
    {
        if((s_required_mask & (1U << i)) == 0U)
            continue;

        const float count = (float)s_acc[i].count;
        const float roll_mean = s_acc[i].roll_sum / count;
        const float pitch_mean = s_acc[i].pitch_sum / count;

        float roll_variance = s_acc[i].roll_sum_sq / count - roll_mean * roll_mean;
        float pitch_variance = s_acc[i].pitch_sum_sq / count - pitch_mean * pitch_mean;

        // 浮点舍入可能使理论上的零方差略小于零。
        if(roll_variance < 0.0f)
            roll_variance = 0.0f;
        if(pitch_variance < 0.0f)
            pitch_variance = 0.0f;

        /*
         * 标准差过大说明整个采样窗口姿态不够稳定，
         * 即使单帧均通过静止门限，也拒绝本次校准结果。
         */
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
    const uint8_t all_imu_mask = 
        (uint8_t)((1U << (uint8_t)ICM_INSTANCE_MAX) - 1U);

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

    /*
     * 从 Config Flash 加载最近一份合法 Level Trim。
     * 
     * 当前 Required IMU 必须全部具有有效 Trim 才整体启用，
     * 避免双 IMU 切换时因只有一侧进行了补偿而产生姿态基准跳变。
     */
    if(BSP_ConfigFlash_LoadLatest(
            LEVEL_TRIM_CONFIG_RECORD_TYPE,
            LEVEL_TRIM_CONFIG_VERSION,
            &stored,
            sizeof(stored),
            NULL) == CONFIG_FLASH_OK &&
        LevelTrim_ConfigValid(&stored) &&
        s_required_mask != 0U &&
        (stored.valid_mask & s_required_mask) == s_required_mask)
    {
        s_config = stored;
    }
}

void LevelTrim_HandleRc(const RCChannelData_t *rc)
{
    if(rc == NULL)
        return;

    const bool se_released = 
        rc->channels[LEVEL_TRIM_RC_CHANNEL_SE] <= LEVEL_TRIM_SE_RELEASED_MAX;

    /*
     * SE 完全释放后解除 Trigger Latch。
     * 因此一次长按只能触发一次校准，必须先松开才能再次触发。
     */
    if(se_released)
    {
        s_trigger_holding = false;
        s_trigger_latched = false;
        return;
    }

    if(s_state != LEVEL_TRIM_STATE_IDLE || s_trigger_latched)
        return;

    /*
     * Level Trim 仅允许在安全静止环境下触发。
     * Mag Calibration 与 Level Trim 不能并行运行。
     */
    const bool eligible =
        rc->link_ok &&
        g_arm_state == ARM_STATE_DISARMED &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_ARM] <= LEVEL_TRIM_ARM_OFF_MAX &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_THROTTLE] <= LEVEL_TRIM_THROTTLE_LOW_MAX &&
        rc->channels[LEVEL_TRIM_RC_CHANNEL_SE] >= LEVEL_TRIM_SE_PRESSED_MIN &&
        !MagCalibration_IsActive() &&
        ImuCalibration_IsReady();

    if(!eligible)
    {
        s_trigger_holding = false;
        return;
    }

    const uint32_t now = osKernelGetTickCount();

    // 首次满足触发条件时建立长按计时基准。
    if(!s_trigger_holding)
    {
        s_trigger_holding = true;
        s_trigger_start_tick = now;
        return;
    }

    if((now - s_trigger_start_tick) >= LEVEL_TRIM_TRIGGER_HOLD_MS)
    {
        s_trigger_holding = false;

        // 必须先释放 SE，才允许下一次 Level Trim。
        s_trigger_latched = true;   

        LevelTrim_Start();
    }
}

void LevelTrim_UpdateSamples(const IcmData_t *imu1, bool imu1_fresh,
                            const IcmData_t *imu2, bool imu2_fresh)
{
    const uint32_t now = osKernelGetTickCount();

    /*
     * 新 Trim 写入并立即生效后，
     * 给姿态互补滤波器一段重新收敛时间，
     * 期间 LevelTrim_IsActive() 仍保持 true，因此禁止解锁。
     */
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
    
    /*
     * 校准过程中若重新 Armed，
     * 或整个采样过程超过最大时间限制，则直接判定失败。
     */
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
            /*
             * 任一 Required IMU 检测到运动或姿态条件异常时，
             * 两颗 IMU 的累计窗口同时清空，
             * 保证最终 Trim 来自同一连续静止区间。
             */
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

    /*
     * 先将新 Trim 持久化到 Config Flash。
     * Flash 写入成功后才替换 RAM 中当前生效配置，
     * 避免运行态配置与持久化状态不一致。
     */
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

    /*
     * 新 Trim 从下一次 Accel Observation 起立即生效，
     * 之后等待滤波器重新再允许解除校准状态。
     */
    s_settle_start_tick = now;
    s_state = LEVEL_TRIM_STATE_FILTER_SETTLING;
}

void LevelTrim_Apply(
    IcmInstance_t instance, 
    float *accel_roll_rad, 
    float *accel_pitch_rad)
{
    const uint8_t index = (uint8_t)instance;

    if(index >= (uint8_t)ICM_INSTANCE_MAX || 
        accel_roll_rad == NULL || 
        accel_pitch_rad == NULL ||
        (s_config.valid_mask & (1U << index)) == 0U)
    {
        return;
    }

    /*
     * Trim 表示机体物理水平时由安装误差产生的 Accel 姿态偏置。
     * 因此从当前 Gravity Observation 中直接扣除。
     */
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
    /*
     * 所有当前 Required IMU 都必须存在有效 Trim，
     * 才允许整个 Level Trim 模块被视为 Ready。
     */
    return s_required_mask != 0U &&
           (s_config.valid_mask & s_required_mask) == s_required_mask;
}

