/**
 * @file    app_imu2_redundancy.c
 * @brief   双 IMU 冗余，健康检测，故障切换及控制周期测量实现。
 */

#include "app_imu2_redundancy.h"
#include "app_imu_calibration.h"
#include "app_level_trim.h"
#include "app_shared_types.h"
#include "bsp_debug_uart.h"
#include "bsp_blackbox.h"
#include <math.h>
#include "cmsis_os2.h"

extern osMessageQueueId_t IndicatorEventQueueHandle;

#define IMU_FAULT_REPORT_MS 900U // Dual Fault 持续期间的故障提示重发周期，ms。

/*
 * IMU 健康状态迟滞门限。
 * 故障切换门限较短，恢复门限较长，避免故障边界附近频繁切换。
 */
#define SWITCH_AWAY_THRESHOLD 5         // 连续异常达到该数据帧后判定 Active IMU 失效。
#define SWITCH_BACK_THRESHOLD 200       // 连续健康达到该帧数后恢复 Standby Healthy 标志。
#define DUAL_FAULT_CLEAR_THRESHOLD 50   // 连续健康达到该帧数后解除 Dual Fault。

/*
 * 双 IMU 六轴 CrossCheck 门限。
 * 
 * 正式值应根据静态数据分布和实际分型动作数据统计确定；
 * 当前数值仍属于测试阶段占位门限。
 */
#define ACC_AX_DIFF_THRESHOLD_G 99.0f
#define ACC_AY_DIFF_THRESHOLD_G 99.0f
#define ACC_AZ_DIFF_THRESHOLD_G 99.0f
#define GYRO_GX_DIFF_THRESHOLD_DPS 99.9f
#define GYRO_GY_DIFF_THRESHOLD_DPS 99.9f
#define GYRO_GZ_DIFF_THRESHOLD_DPS 99.9f

/*
 * ICM ODR = 800 Hz，对应周期约为 1.25ms。
 * DRDY Watchdog 取约 3 个采样周期并向上覆盖到 RTOS Tick。
 * 
 * 该超时只用于判断数据是否长期未刷新；
 * 实际控制 Dt 必须由 DWT Cycle Counter 测量。
 */
#define ICM_DRDY_WAIT_TIMEOUT_MS 4

/* IMU Cache 允许的最大数据年龄，约 3 个 ODR 周期。 */
static const float MAX_AGE_S = 3.0f / ICM_ODR_HZ;

/**
 * @brief   Active IMU 时间戳检查结果。
 */
typedef enum
{
    ACTIVE_DT_OK = 0,       /**< Active IMU 产生新帧，Dt 有效。 */
    ACTIVE_DT_DUPLICATE,    /**< Active 时间戳未变化，本轮没有新的 Active 帧。 */
    ACTIVE_DT_ANOMALY       /**< Active 有新帧，但 Dt 超出可信范围。 */
} ActiveDtResult_t;

static uint32_t s_last_active_cycle;        // 上一有效 Active IMU 帧的 DWT 时间戳。
static uint8_t s_last_active_sel = 0xFFU;   // 上一控制周期使用的 Active IMU 编号；0xFF 表示尚未建立。
static bool s_active_timestamp_valid;       // 当前 Active IMU 是否已经建立有效时间戳基准。
static uint8_t s_available_mask;    // 启动时 WHO_AM_I 初始化成功，实际可参与冗余管理的 IMU 位掩码。

/*
 * 根据 Active IMU 的 DWT 时间戳计算实际控制周期。
 * 
 * IMU 切换后的第一帧不跨不同传感器计算 Dt，而使用标称周期重新建立基准。
 * 时间戳不变表示本轮只刷新了 Standby IMU。
 */
static ActiveDtResult_t ActiveFrame_GetDt(uint8_t active_sel, uint32_t timestamp_cycle, float *dt_s)
{
    const float nominal_dt = 1.0f / (float)ICM_ODR_HZ;

    /*
     * Active IMU 发生切换后，两颗传感器的上一采样时刻没有连续关系，
     * 因此丢弃时间基准，从当前传感器重新建立。
     */
    if(active_sel != s_last_active_sel)
    {
        s_last_active_sel = active_sel;
        s_active_timestamp_valid = false;
    }

    if(!s_active_timestamp_valid)
    {
        s_last_active_cycle = timestamp_cycle;
        s_active_timestamp_valid = true;
        *dt_s = nominal_dt;
        return ACTIVE_DT_OK;
    }

    uint32_t delta_cycle = timestamp_cycle - s_last_active_cycle;

    // 时间戳没变化，说明仍是上一次 active IMU 缓存
    if(delta_cycle == 0U)
    {
        return ACTIVE_DT_DUPLICATE;
    }

    /*
     * 无论当前 Dt 是否异常，都立即更新采样基准。
     * 否则下一帧仍会继续包含本次异常间隔。
     */
    s_last_active_cycle = timestamp_cycle;

    float measured_dt = (float)delta_cycle / (float)SystemCoreClock;

    /*
     * 标称 800 Hz Dt 约为 1.25ms。
     * 当前允许少量丢帧，但拒绝明显异常的过段或过长间隔。
     */
    if((measured_dt < 0.0005f) || (measured_dt > 0.0050f))
    {
        *dt_s = nominal_dt;
        return ACTIVE_DT_ANOMALY;
    }

    *dt_s = measured_dt;
    return ACTIVE_DT_OK;
}

/*
 * 启用 Cortex-M DWT Cycle Counter。
 * 
 * DWT->CYCCNT 提供 CPU Cycle 级时间戳，
 * 用于测量真实 IMU Sample Dt，而不是依赖 ms 级 RTOS Tick。
 */
static void DWT_Init(void)
{
    // 开启 Cortex-M Trace 单元，使 DWT 寄存器可用。
    CoreDebug->DEMCR 
        |= CoreDebug_DEMCR_TRCENA_Msk;

    // 从零开始运行 32-bit Cycle Counter。
    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
 * 对两颗 IMU 的六轴测量执行 CrossCheck。
 * 
 * 任一轴差值超过对应门限即认为两颗传感器当前观测不一致。
 * 本函数只判断“两者是否存在异常差异”，不负责判定具体故障来源。
 */
static bool CrossCheck_IsAbnormal(const IcmData_t *a, const IcmData_t *b)
{
    if (fabsf(a->ax - b->ax) > ACC_AX_DIFF_THRESHOLD_G)
        return true;
    if (fabsf(a->ay - b->ay) > ACC_AY_DIFF_THRESHOLD_G)
        return true;
    if (fabsf(a->az - b->az) > ACC_AZ_DIFF_THRESHOLD_G)
        return true;
    if (fabsf(a->gx - b->gx) > GYRO_GX_DIFF_THRESHOLD_DPS)
        return true;
    if (fabsf(a->gy - b->gy) > GYRO_GY_DIFF_THRESHOLD_DPS)
        return true;
    if (fabsf(a->gz - b->gz) > GYRO_GZ_DIFF_THRESHOLD_DPS)
        return true;
    return false;
}

/*
 * 记录一次当前冗余观测异常。
 * 
 * 连续异常达到 SWITCH_AWAY_THRESHOLD 后，
 * 若 Standby 仍被认为健康，则切换 Active IMU；
 * 若 Standby 已不健康，则锁存 Dual Fault，不再无意义地来回切换。
 */
static void Health_RecordBad(void)
{
    g_imu_health.good_frame_count = 0;

    if(g_imu_health.bad_frame_count < 0xFFFF)
        g_imu_health.bad_frame_count++;
    
    if(g_imu_health.bad_frame_count >= SWITCH_AWAY_THRESHOLD)
    {
        uint8_t standby_healthy = (g_imu_health.active_imu_sel == 0
                                       ? g_imu_health.imu2_healthy
                                       : g_imu_health.imu1_healthy);
        
        if(standby_healthy)
        {
            /*
             * Standby 仍被认为健康时，将当前 Active 标记异常，
             * 并切换到另一颗 IMU。
             */
            if (g_imu_health.active_imu_sel == 0U)
            {
                g_imu_health.imu1_healthy = 0U;
                g_imu_health.active_imu_sel = 1U;
            }
            else
            {
                g_imu_health.imu2_healthy = 0U;
                g_imu_health.active_imu_sel = 0U;
            }

            g_imu_health.dual_fault = 0U;

            BB_LogImuSwitch(osKernelGetTickCount(), g_imu_health.active_imu_sel);
        }
        else
        {
            /*
             * Stanby 已经不健康时，连续切换没有意义，
             * 因此保持当前 Active 选择不变，并锁存 Dual Fault。
             * 
             * 本层只报告冗余状态，不直接决定停桨等上层安全动作。
             */
            if(g_imu_health.active_imu_sel == 0)
            {
                g_imu_health.imu1_healthy = 0;
            }
            else
            {
                g_imu_health.imu2_healthy = 0;
            }

            // Dual Fault 仅在首次进入时写入一次 Blackbox 事件。
            if(!g_imu_health.dual_fault) 
                BB_LogDualFault(osKernelGetTickCount());

            g_imu_health.dual_fault = 1;
        }

        /*
         * 完成一次故障状态处理后重新开始健康/异常连续计数，
         * 避免同一累计窗口被重复消费。
         */
        g_imu_health.bad_frame_count = 0;
        g_imu_health.good_frame_count = 0;
    }
}

/*
 * 记录一次当前冗余观测健康。
 * 
 * 连续健康用于恢复 IMU Healthy 标志和解除 Dual Fault，
 * 恢复门限明显高于切走门限，以形成故障状态迟滞。
 */
static void Health_RecordGood(void)
{
    g_imu_health.bad_frame_count = 0;

    if(g_imu_health.good_frame_count < 0xFFFF)
        g_imu_health.good_frame_count++;
    
    // 当前 active IMU 持续正常，标志维持 healthy。
    if(g_imu_health.active_imu_sel == 0)
    {
        g_imu_health.imu1_healthy = 1;
    }
    else
    {
        g_imu_health.imu2_healthy = 1;
    }

    /*
     * Active IMU 连续稳定一段时间后，
     * 说明至少已经恢复一路可信测量，可解除 Dual Fault 锁存。
     */
    if (g_imu_health.dual_fault && 
        g_imu_health.good_frame_count >= DUAL_FAULT_CLEAR_THRESHOLD)
    {
        g_imu_health.dual_fault = 0;
    }
    
    /*
     * 连续健康达到更长门限后，恢复 Standby IMU 的 Healthy 标志。
     * 
     * 这里只恢复健康姿态，不立即切回，
     * 防止冗余模块在两颗健康 IMU 之间无意义往返切换。
     */
    if(g_imu_health.good_frame_count >= SWITCH_BACK_THRESHOLD)
    {
        if(g_imu_health.active_imu_sel == 0 &&
            (s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U)
        {
            g_imu_health.imu2_healthy = 1;
        }
        else if(g_imu_health.active_imu_sel == 1 &&
            (s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U)
        {
            g_imu_health.imu1_healthy = 1;
        }
    }
}

/*
 * Dual Fault 持续期间向 Indicator Task 发布故障事件。
 * 
 * 首次进入立即上报；若故障持续存在，则按固定周期重发，
 * 避免一次提示播放结束后故障仍然没有可见状态。 
 */
static void ImuFault_ReportIfActive(void)
{
    static bool was_active = false;
    static uint32_t last_repost_tick = 0;

    const bool active = (g_imu_health.dual_fault != 0);

    if(active)
    {
        const uint32_t now = osKernelGetTickCount();

        if (!was_active || 
            (now - last_repost_tick) >= IMU_FAULT_REPORT_MS)
        {
            IndicatorEvent_t evt = EVT_IMU_FAULT;

            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);

            last_repost_tick = now;
        }
    }

    was_active = active;
}

void ImuRedundancy_Init(uint8_t init_fail_mask)
{
    DWT_Init();

    s_last_active_cycle = 0U;
    s_last_active_sel = 0xFFU;
    s_active_timestamp_valid = false;

    /*
     * init_fail_mask 中置位表示对应 IMU 初始化失败，
     * 因此取反后只保留当前实现支持的两颗 IMU 位。
     */
    s_available_mask =
        (uint8_t)(~init_fail_mask &
                  (IMU_CAL_REQUIRED_IMU1 |
                   IMU_CAL_REQUIRED_IMU2));

    g_imu_health.imu1_healthy = (s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U;
    g_imu_health.imu2_healthy = (s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U;

    /*
     * 有限使用 IMU1 作为初始 Active；
     * IMU1 不可用时退化到 IMU2。
     */
    g_imu_health.active_imu_sel = g_imu_health.imu1_healthy ? 0U : 1U;

    g_imu_health.bad_frame_count = 0U;
    g_imu_health.good_frame_count = 0U;

    g_imu_health.dual_fault = (s_available_mask == 0U) ? 1U : 0U;

    /*
     * 只有 WHO_AM_I 初始化成功的 IMU 才参与启动校准和 Level Trim。
     * Gyro Bias 只在本次运行期间有效，不写入 Flash。
     */
    ImuCalibration_Init(s_available_mask);
    LevelTrim_Init(s_available_mask);
}

/*
 * 执行一次 双 IMU 冗余更新。
 * 
 * 处理顺序：
 * DRDY Wait -> SPI Refresh -> Gyro Calibration -> Bias Apply ->
 * Level Trim Sampling -> CrossCheck -> Health Update ->
 * Active Select -> Dt Validation。
 */
ImuUpdateResult_t ImuRedundancy_Update(IcmData_t *out, float *dt_s, uint8_t *fresh_flags)
{
    if((out == NULL) || (dt_s == NULL))
    {
        return IMU_UPDATE_TIMEOUT;
    }

    /*
     * 任意一颗 IMU 的 DRDY 均可唤醒当前 Task。
     * 等待时间只承担 Watchdog 功能，不用与控制周期测量。
     */
    uint32_t evt = osEventFlagsWait(g_icmDataReadyEvtId,
                                    ICM1_DRDY_FLAG | ICM2_DRDY_FLAG,
                                    osFlagsWaitAny,
                                    ICM_DRDY_WAIT_TIMEOUT_MS);
    
    /*
     * CMSIS-RTOS 2 EventFlags 错误码按负值解释。
     * 两路 DRDY 均未在 Watchdog 窗口内到达时记一次异常。
     */
    if((int32_t)evt < 0)
    {
        if (fresh_flags != NULL)
        {
            *fresh_flags = 0U;
        }

        Health_RecordBad();
        ImuFault_ReportIfActive();

        return g_imu_health.dual_fault
                   ? IMU_UPDATE_DUAL_FAULT
                   : IMU_UPDATE_TIMEOUT;
    }

    uint8_t fresh = (uint8_t)(evt & (ICM1_DRDY_FLAG | ICM2_DRDY_FLAG));

    if(fresh_flags != NULL)
    {
        *fresh_flags = fresh;
    }

    IcmData_t d1, d2;

    const bool got1 = (evt & ICM1_DRDY_FLAG) != 0;
    const bool got2 = (evt & ICM2_DRDY_FLAG) != 0;

    /*
     * 仅对本轮产生的 DRDY 的 IMU 触发新读取；
     * 随后统一复制两颗 IMU Cache，便于 CrossCheck 和新鲜度判断。
     */
    if(got1)
        ICM_TriggerRead(ICM_INSTANCE_1);
    if(got2)
        ICM_TriggerRead(ICM_INSTANCE_2);

    ICM_CopyTo(ICM_INSTANCE_1, &d1);
    ICM_CopyTo(ICM_INSTANCE_2, &d2);

    /*
     * 仅将“已成功初始化且本轮确实有新数据”的 IMU 样本
     * 提交给启动 Gyro Bias Calibration。
     */
    const bool cal_ready = ImuCalibration_Update(
        &d1, got1 && ((s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U),
        &d2, got2 && ((s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U));
    
    if(!cal_ready)
    {
        return IMU_UPDATE_CALIBRATION;
    }

    /*
     * CrossCheck，姿态估计与控制器统一使用去除启动 Gyro Bias 后的数据，
     * 避免固定零偏直接表现为两颗 IMU 的长期差异。
     */
    if((s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U)
        ImuCalibration_Apply(ICM_INSTANCE_1, &d1);
    
    if((s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U)
        ImuCalibration_Apply(ICM_INSTANCE_2, &d2);

    /*
     * Level Trim 使用校准后的 IMU 数据持续维护自身样本状态，
     * 但只把本轮真正 Fresh 的传感器标记为新样本。
     */
    LevelTrim_UpdateSamples(
        &d1, got1 && ((s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U),
        &d2, got2 && ((s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U));

    // DebugUart_PrintImuDiff(&d1, &d2);

    /*
     * 使用 DWT 时间戳检查两颗可用 IMU Cache 的数据年龄。
     * 即使本轮只有一颗产生 DRDY，也能是被另一颗是否已经长期未刷新。
     */
    uint32_t now_cycle = DWT->CYCCNT;

    float age1_s = (float)(now_cycle - d1.timestamp_cycle) / (float)SystemCoreClock;
    float age2_s = (float)(now_cycle - d2.timestamp_cycle) / (float)SystemCoreClock;

    bool age_bad =
        (((s_available_mask & IMU_CAL_REQUIRED_IMU1) != 0U) && age1_s > MAX_AGE_S) ||
        (((s_available_mask & IMU_CAL_REQUIRED_IMU2) != 0U) && age2_s > MAX_AGE_S);

    if (age_bad)
    {
        Health_RecordBad();
    }
    else if (s_available_mask == 
            (IMU_CAL_REQUIRED_IMU1 | IMU_CAL_REQUIRED_IMU2) && 
            CrossCheck_IsAbnormal(&d1, &d2))
    {
        /*
         * 只有两颗 IMU 都实际存在时才执行 CrossCheck。
         * 单 IMU 降级模式无法通过相互比较判断测量一致性。
         */
        Health_RecordBad();
    }
    else
    {
        Health_RecordGood();
    }

    ImuFault_ReportIfActive();

    if(g_imu_health.dual_fault)
    {
        return IMU_UPDATE_DUAL_FAULT;
    }

    const uint8_t active_sel = g_imu_health.active_imu_sel;
    const IcmData_t *active_data = (active_sel == 0U) ? &d1 : &d2;

    /*
     * 只有 Active IMU 时间戳真正推进，
     * 才允许本轮作为新的 Flight Control Sample。
     */
    const ActiveDtResult_t dt_result = ActiveFrame_GetDt(active_sel, active_data->timestamp_cycle, dt_s);

    if(dt_result == ACTIVE_DT_DUPLICATE)
    {
        return IMU_UPDATE_STANDBY_ONLY;
    }

    /*
     * Active 有新帧时先输出其完整数据。
     * 即使 Dt 异常，调用方仍可获得对应帧用于诊断，
     * 但必须根据返回状态跳过正常姿态积分与 PID。
     */
    *out = *active_data;

    if(dt_result == ACTIVE_DT_ANOMALY)
    {
        return IMU_UPDATE_TIMING_ANOMALY;
    }
    
    return IMU_UPDATE_ACTIVE_FRAME;
}
