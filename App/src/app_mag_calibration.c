/**
 * @file    app_mag_calibration.c
 * @brief   Mag Calibration 原始数据采集及运行时 Hard/Soft-Iron 校正实现。
 * 
 * 本模块包含两个相互独立的职责：
 * 
 * 1. 通过 RC 手势控制原始 Mag Calibration 数据采集，
 *      并借助 Blackbox 文件保存数据，供后续离线拟合；
 * 
 * 2. 在正常运行时使用已拟合的 Hard-Iron Bias 和
 *      Soft-Iron Matrix 对 Mag 向量进行校正。
 * 
 * 当前 MCU 端不执行椭球拟合，采集完成也不会自动更新
 * s_mag_calibration 中的参数。
 */

#include "app_mag_calibration.h"
#include "app_arm.h"
#include "app_blackbox.h"
#include "app_imu_calibration.h"
#include "app_level_trim.h"
#include "app_rc_calibration.h"
#include "app_shared_types.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

/* RC 通道与三段 SB 位置定义。 */
#define MAG_CAL_RC_CHANNEL_SB 7U        // SB：CRSF CH8，对应 channels[7]。
#define MAG_CAL_RC_CHANNEL_ARM 4U       // ARM 开关通道。
#define MAG_CAL_RC_CHANNEL_THROTTLE 2U  // Throttle 通道。

#define MAG_CAL_SB_LOW_MAX 400U         // SB Low 最大通道值。
#define MAG_CAL_SB_MIDDLE_MIN 700U      // SB Middle 最小通道值。
#define MAG_CAL_SB_MIDDLE_MAX 1300U     // SB Middle 最大通道值。
#define MAG_CAL_SB_HIGH_MIN 1600U       // SB High 最小通道值。

#define MAG_CAL_ARM_OFF_MAX 900U
#define MAG_CAL_THROTTLE_LOW_MAX 200U
#define MAG_CAL_TRIGGER_HOLD_MS 3000U

/*
 * Mag Calibration 原始数据采样数量。
 * QMC 标称 50 Hz 时，1500 样本约为 30s，8000 样本约 160s。
 */
#define MAG_CAL_MIN_SAMPLE_COUNT 1500U  // 50Hz下约为 30s
#define MAG_CAL_MAX_SAMPLE_COUNT 8000U  // 50Hz下约为 160s

/**
 * @brief   运行时使用的 Mag Calibration 参数。
 */
typedef struct 
{
    float bias[3];      /**< Hard-Iron Bias，N/E/D 三轴，Gauss。 */

    /* 
     * 对称 Soft-Iron 矩阵的六个独立元素：
     * A00, A01, A02, A11, A12, A22 
     */
    float soft_matrix_sym[6];

    /*
     * 拟合环境中得到的校正后参考磁场模长，Gauss。
     * 
     * 不直接参与向量校正，
     * 主要用于运行时磁场有效性判断与诊断。
     */
    float field_reference_gauss;
} MagCalibrationConfig_t;

/*
 * 当前运行时使用的离线拟合结果。
 * 
 * Mag 数据采集流程本身不会修改这些参数；
 * 新采样的数据需要在外部完成拟合后再更新此配置。
 */
static const MagCalibrationConfig_t s_mag_calibration =
{
    .bias = {
        0.769966473f,
        -1.132992138f,
        0.907445057f
    },
    .soft_matrix_sym = {
        1.007267649f,
        0.029346188f,
        0.008633474f,
        0.990599558f,
        0.004140657f,
        1.003350263f
    },
    .field_reference_gauss = 0.473539573f,
};

/**
 * @brief Mag Calibration 数据采集状态。
 */
typedef enum
{
    MAG_CAL_STATE_IDLE = 0,        /**< 空闲，等待新的 RC 触发。 */
    MAG_CAL_STATE_WAIT_FILE_OPEN,  /**< 已请求新文件，等待 Blackbox Task 完成打开。 */
    MAG_CAL_STATE_CAPTURING,       /**< 正在记录原始 Mag 样本。 */
    MAG_CAL_STATE_WAIT_FILE_CLOSE, /**< 已停止生产样本，等待 Blackbox Flush 并关闭文件。 */
} MagCalibrationState_t;

static MagCalibrationState_t s_state = MAG_CAL_STATE_IDLE;

/* RC 长按触发状态。 */
static bool s_trigger_holding;
static bool s_trigger_latched;
static uint32_t s_trigger_start_tick;

/*
 * 等待 Blackbox 文件打开期间若触发条件失效，
 * 无法撤销已经发出的 New File Request，
 * 因此记录 Abort请求，待文件真正打开后立即关闭。
 */
static bool s_abort_when_open;

/*
 * 文件关闭前冻结本次采集是否具备“成功候选”资格。
 * 最终仍需等待 Blackbox 确认文件确实成功关闭。
 */
static bool s_finish_candidate_success;


/*
 * 跨 Task 共享状态：
 * - s_capture_enabled: FlightCtrl 设置，Nav 根据它决定是否记录 Mag Sample。
 * - s_sample_count: Nav 在成功记录 Sample 后累加，FlightCtrl 用它判断采样是否足够。
 */
static volatile bool s_capture_enabled;
static volatile uint32_t s_sample_count;

/* 非阻塞向 Indicator Task 发布 Mag Calibration 状态事件。 */
static void MagCalibration_PostEvent(IndicatorEvent_t evt)
{
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}

/* 判断 SB 是否处于 Low 档。 */
static bool MagCalibration_SbIsLow(uint16_t value)
{
    return value <= MAG_CAL_SB_LOW_MAX;
}

/* 判断 SB 是否处于 Middle 档。 */
static bool MagCalibration_SbIsMiddle(uint16_t value)
{
    return value <= MAG_CAL_SB_MIDDLE_MAX &&
           value >= MAG_CAL_SB_MIDDLE_MIN;
}

/* 判断 SB 是否处于 High 档。 */
static bool MagCalibration_SbIsHigh(uint16_t value)
{
    return value >= MAG_CAL_SB_HIGH_MIN;
}

/*
 * 请求结束当前 Mag 数据采集。
 * 
 * 必须先禁止 Nav Task 继续生产 Calibration Sample，
 * 再通知 Blackbox Task Flush 剩余 Buffer 并关闭文件。
 * 
 * success 仅表示调用房希望按“正常结束”处理；
 * 最终成功还要求样本数足够，Blackbox 无错误且文件关闭成功。
 */
static void MagCalibration_RequestFinish(bool success)
{
    s_capture_enabled = false;

    s_finish_candidate_success = 
        success &&
        s_sample_count >= MAG_CAL_MIN_SAMPLE_COUNT &&
        BB_GetErrorFlags() == 0U;

    s_state = MAG_CAL_STATE_WAIT_FILE_CLOSE;
    BB_RequestClose();
}

void MagCalibration_Init(void)
{
    s_state = MAG_CAL_STATE_IDLE;

    s_trigger_holding = false;
    s_trigger_latched = false;

    s_abort_when_open = false;
    s_finish_candidate_success = false;

    s_trigger_start_tick = 0U;

    s_capture_enabled = false;
    s_sample_count = 0U;
}

void MagCalibration_HandleRc(const RCChannelData_t *rc)
{
    if(rc == NULL)
        return;

    const uint16_t sb = rc->channels[MAG_CAL_RC_CHANNEL_SB];
    const bool sb_low = MagCalibration_SbIsLow(sb);
    const bool sb_middle = MagCalibration_SbIsMiddle(sb);
    const bool sb_high = MagCalibration_SbIsHigh(sb);

    /*
     * 已向 Blackbox 请求创建文件，但真正的 File Open
     * 由较低优先级 Blackbox Task 异步完成。
     * 
     * 此阶段始终保持 Calibration Active，禁止 Arm。
     */
    if(s_state == MAG_CAL_STATE_WAIT_FILE_OPEN)
    {
        /*
         * 请求发出后无法直接撤销。
         * 若等待过程中 RC 失联，重新 Armed 或 SB 离开 High，
         * 记录 Abort，等文件实际打开后立即关闭。
         */
        if(!rc->link_ok || 
            g_arm_state != ARM_STATE_DISARMED ||
            sb_low || 
            sb_middle)
        {
            s_abort_when_open = true;
        }

        if(App_Blackbox_IsFileOpen())
        {
            if(s_abort_when_open)
            {
                MagCalibration_RequestFinish(false);
            }
            else
            {
                /*
                 * 文件已经确认打开后才允许 Nav Task 写入 Mag Sample，
                 * 防止采集前几帧落在文件尚未建立的窗口中。
                 */
                s_capture_enabled = true;
                s_state = MAG_CAL_STATE_CAPTURING;

                MagCalibration_PostEvent(EVT_MAG_CAL_CAPTURE_STARTED);
            }
        }
        return;
    }

    if(s_state == MAG_CAL_STATE_CAPTURING)
    {
        /*
         * RC 失联，重新 Armed 或 SB 切到 Low
         * 均视为异常终止，本次采集判定失败。
         */
        if(!rc->link_ok || 
            g_arm_state != ARM_STATE_DISARMED || 
            sb_low)
        {
            MagCalibration_RequestFinish(false);
        }
        /*
         * High -> Middle 表示飞手主动结束采集。
         * 样本数量和 Blackbox 状态将在 RequestFinish 中再次检查。
         */
        else if(sb_middle)
        {
            MagCalibration_RequestFinish(true);
        }
        /*
         * 达到最大样本数量后自动停止，
         * 防止忘记结束采集导致文件无限增大。
         */
        else if(s_sample_count >= MAG_CAL_MAX_SAMPLE_COUNT)
        {
            MagCalibration_RequestFinish(true);
        }
        /*
         * Blackbox 在采集中出现 Buffer/SD 错误时，
         * 当前数据集已不再可信，立即按失败路径结束。
         */
        else if(BB_GetErrorFlags() != 0U)
        {
            MagCalibration_RequestFinish(false);
        }
        return;
    }

    if(s_state == MAG_CAL_STATE_WAIT_FILE_CLOSE)
    {
        /*
         * Blackbox Task 负责 Flush 尾部数据并真正关闭文件。
         * 文件完全关闭后，才能确定本次采集最终结果。
         */
        if(!App_Blackbox_IsFileOpen())
        {
            const bool success = s_finish_candidate_success &&
                                 App_Blackbox_LastCloseSucceeded();

            MagCalibration_PostEvent(
                success 
                    ? EVT_MAG_CAL_CAPTURE_DONE 
                    : EVT_MAG_CAL_CAPTURE_FAILED);

            s_state = MAG_CAL_STATE_IDLE;

            s_abort_when_open = false;
            s_finish_candidate_success = false;
            s_sample_count = 0U;

            /*
             * （应对达到样本上限自动关闭的情况）
             * trigger_latched 保持 true。
             * 必须先将 SB 拨回 Middle，才能重新使能下一次重新采集触发。
             *
             * 对应下方第一处 “if(!sb_high || s_trigger_latched)” 分支。
             */
        }

        return;
    }

    /*
     * 以下仅处理 IDLE。
     * 
     * Middle 是唯一的 Trigger Rearm 位置：
     * 一次 Calibration 完成后必须先回到 Middle，
     * 才允许再次通过 High 长按触发。
     */
    if(sb_middle)
    {
        s_trigger_holding = false;
        s_trigger_latched = false;
        return;
    }

    if(!sb_high || s_trigger_latched)
    {
        s_trigger_holding = false;
        return;
    }

    /*
     * Mag Calibration 只允许在安全且无其他校准流程冲突时重启。
     * 
     * 同时要求 Blackbox 当前没有打开文件。
     * 因此本次采集需要独占一个新的 Calibration Log File。
     */
    const bool eligible =
        rc->link_ok &&
        g_arm_state == ARM_STATE_DISARMED &&
        rc->channels[MAG_CAL_RC_CHANNEL_ARM] <= MAG_CAL_ARM_OFF_MAX &&
        rc->channels[MAG_CAL_RC_CHANNEL_THROTTLE] <= MAG_CAL_THROTTLE_LOW_MAX &&
        ImuCalibration_IsReady() &&
        !RcCalibration_IsActive() &&
        !LevelTrim_IsActive() &&
        !App_Blackbox_IsFileOpen();
    
    if(!eligible)
    {
        s_trigger_holding = false;
        return;
    }

    const uint32_t now = osKernelGetTickCount();

    // 首次满足 High + Eligible 条件时建立长按计时基准。
    if(!s_trigger_holding)
    {
        s_trigger_holding = true;
        s_trigger_start_tick = now;
        return;
    }

    /*
     * High 持续达到触发时间后锁存本次触发，
     * 并异步请求 Blackbox 创建新的采集文件。
     */
    if((now - s_trigger_start_tick) >= MAG_CAL_TRIGGER_HOLD_MS)
    {
        s_trigger_holding = false;
        s_trigger_latched = true;
        
        s_abort_when_open = false;
        s_finish_candidate_success = false;

        s_sample_count = 0U;
        s_capture_enabled = false;

        s_state = MAG_CAL_STATE_WAIT_FILE_OPEN;
        
        BB_RequestNewFile();
    }
}

void MagCalibration_LogSample(const MagData_t *mag)
{
    /*
     * 只有文件已经打开并正式进入 Capturing 后才允许写入。
     * QMC Overflow 样本并不参与 Calibration 数据集。
     */
    if(!s_capture_enabled || 
        mag == NULL || 
        mag->ovfl)
    {
        return;
    }

    /*
     * 记录尚未执行 Hard/Soft-Iron 校正的 NED Mag 向量，
     * 供后续离线拟合原始磁场椭球。
     * 
     * 仅在 BB_LogMagCalibration 成功接收该帧后增加 Sample Count。
     */
    if(BB_LogMagCalibration(DWT->CYCCNT,
                            mag->MX,
                            mag->MY,
                            mag->MZ) == 0)
    {
		s_sample_count++;
    }
}

void MagCalibration_Apply(MagData_t *mag)
{
    if(mag == NULL || mag->ovfl)
        return;

    /*
     * 第一步：消除 Hard-Iron Bias。
     * 
     * 因为本函数随后会原地覆盖 MX/MY/MZ，
     * 必须先保存三个去 Bias 后的中间分量。
     */
    const float x = mag->MX - s_mag_calibration.bias[0];
    const float y = mag->MY - s_mag_calibration.bias[1];
    const float z = mag->MZ - s_mag_calibration.bias[2];

    const float *a = s_mag_calibration.soft_matrix_sym;

    /* 
     * 第二步：应用 Soft-Iron 校正矩阵：
     * 
     * corrected = A * (raw_ned - bias)
     * 
     * A为对称矩阵：
     * 
     *     [ A00 A01 A02 ]
     * A = [ A01 A11 A12 ]
     *     [ A02 A12 A22 ]
     * 
     * soft_matrix_sym 的存储顺序为：
     * A00，A01，A02，A11，A12，A22。 
     */
    mag->MX = a[0] * x + a[1] * y + a[2] * z;
    mag->MY = a[1] * x + a[3] * y + a[4] * z;
    mag->MZ = a[2] * x + a[4] * y + a[5] * z;
}

float MagCalibration_GetFieldReferenceGauss(void)
{
    return s_mag_calibration.field_reference_gauss;
}

bool MagCalibration_IsActive(void)
{
    /*
     * 长按触发阶段本身也禁止 Arm，
     * 防止飞手在已经开始 Calibration 手势后中途解锁。
     */
    return s_trigger_holding || s_state != MAG_CAL_STATE_IDLE;
}
