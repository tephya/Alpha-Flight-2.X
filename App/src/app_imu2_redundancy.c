#include "app_imu2_redundancy.h"
#include "cmsis_os2.h"
#include "app_shared_types.h"

volatile ImuHealthStatus_t g_imu_health = {
    .active_imu_sel = 0,
    .imu1_healthy = 1,
    .imu2_healthy = 1,
    .bad_frame_count = 0,
    .good_frame_count = 0,
};

/*====== 切换阈值 ======*/
#define SWITCH_AWAY_THRESHOLD 5     // 连续5帧异常判定切走
#define SWITCH_BACK_THRESHOLD 200   // 连续200帧健康判定切回

/*====== 交叉对比阈值 ======*/
#define GYRO_DIFF_THRESHOLD_DPS 0.0f
#define ACC_DIFF_THRESHOLD_G 0.0f

/* ODR=800Hz, 周期1.25ms，超时=3倍周期-3.75ms，向上取整到RTOS tick(1ms)为4ms 
 * 注：这是ms级tick，用于故障超时判定精度足够（只是留裕量的看门狗），
 *     不能拿这个tick分辨率去测量dt，dt必须DWT测*/
#define ICM_DRDY_WAIT_TIMEOUT_MS 4

/*--------------- DWT高精度计时，用于测量真实dt -----------------*/
static uint32_t s_last_cycle = 0;
static bool s_dwt_inited = false;

/**
 * @brief   开启内核里的DWT Cycle Counter(CPU周期计数器)
 * @note    DWT 属于 Cortex-M 的调试/追踪单元（Debug and Trace Unit），默认关闭
 *          DEMCR: Debug Exceptor and Monitor Control Register
 */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // 开启Trace功能模块
    // 清零DWT内部的32位向上计数器
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_inited = true;
}

/**
 * @brief   返回距上次调用的真实间隔（秒），并做钳位：超出标称周期±20%视为调度异常帧
 * @note    用标称周期兜底而不是把失真的dt直接喂给PID
 */
static float DWT_MeasureDt(void)
{
    if(!s_dwt_inited){
        DWT_Init();
        s_last_cycle = DWT->CYCCNT;
        return 1.0f / ICM_ODR_HZ;       // 第一次调用没有上次基准，返回标称值
    }

    uint32_t now = DWT->CYCCNT;
    uint32_t delta_cycles = now - s_last_cycle;     // 依赖无符号溢出自动处理，168MHz下约25s绕回一次

    s_last_cycle = now;

    float dt = (float)delta_cycles / (float)SystemCoreClock;        // x * T

    const float nominal = 1.0f / ICM_ODR_HZ;        // 前后两次控制环执行间隔时间经验值（后续可调整）
    const float lo = nominal * 0.8f;
    const float hi = nominal * 1.2f;
    if(dt < lo || dt > hi){             // 如果实测值不在这个经验值±20%上波动，
        dt = nominal;
    }
    return dt;
} 

/**
 * @brief   交叉比对两个ICM测量的数据
 * @param   a ICM1实体
 * @param   b ICM2实体
 * @retval  false(未确定阈值前暂时的返回值)
 */
static bool CrossCheck_IsAbnormal(const IcmData_t *a, const IcmData_t *b)
{
    (void)a;
    (void)b;
    /*TODO: 确定阈值后启用*/
    return false;
}

/**
 * @brief   ICM错误计数与切换
 */
static void Health_RecordBad(void)
{
    g_imu_health.good_frame_count = 0;
    if(g_imu_health.bad_frame_count < 0xFFFF)
        g_imu_health.bad_frame_count++;
    
    if(g_imu_health.bad_frame_count >= SWITCH_AWAY_THRESHOLD)
    {
        if(g_imu_health.active_imu_sel == 0)
        {
            g_imu_health.imu1_healthy = 0;
            g_imu_health.active_imu_sel = 1;
        }
        else
        {
            g_imu_health.imu2_healthy = 0;
            g_imu_health.active_imu_sel = 0;
        }
        g_imu_health.bad_frame_count = 0;
        g_imu_health.good_frame_count = 0;
    }
}

/**
 * @brief   ICM健康计数
 */
static void Health_RecordGood(void)
{
    g_imu_health.bad_frame_count = 0;
    if(g_imu_health.good_frame_count < 0xFFFF)
        g_imu_health.good_frame_count++;
    
    // 若当前非主用IMU连续健康达到阈值，允许切回（快切慢回）
    uint8_t standby_healthy_flag = (g_imu_health.active_imu_sel == 0)
                                       ? g_imu_health.imu2_healthy
                                       : g_imu_health.imu1_healthy;

    (void)standby_healthy_flag;

    if(g_imu_health.active_imu_sel == 0)
    {
        g_imu_health.imu1_healthy = 1;
    }
    else
    {
        g_imu_health.imu2_healthy = 1;
    }
}

/**
 * @brief   初始化IMU冗余处理模块
 */
void ImuRedundancy_Init(void)
{
    DWT_Init();
}

/**
 * @brief   执行ICM冗余检验逻辑，并拷贝姿态数据给调用方
 * @note    本函数执行包含以下动作：
 *          1. 等待任一IMU的DRDY事件（带超时）
 *          2. 触发SPI读取
 *          3. 交叉比对+健康判定+主备切换
 *          4. 测量真实dt
 * @param   out 调用方提供的接收数据结构体指针
 * @param   dt_s 调用方距离上一次调用的时间间隔
 * @retval  true=本帧数据有效可用于结算
 *          false=本帧超时/双路失效，调用方应跳过本次解算
 */
bool ImuRedundancy_Update(IcmData_t *out, float *dt_s)
{
    uint32_t evt = osEventFlagsWait(g_icmDataReadyEvtId,
                                    ICM1_DRDY_FLAG | ICM2_DRDY_FLAG,
                                    osFlagsWaitAny,
                                    ICM_DRDY_WAIT_TIMEOUT_MS);
    
    if((int32_t)evt < 0)    // CMSIS-RTOS2: 负值为错误码，osFlagsErrorTimeout即超时
    {
        Health_RecordBad();     // 两路都没等到，算一次中断型异常，计入统一计数器
        *dt_s = DWT_MeasureDt();
        return false;
    }

    IcmData_t d1, d2;
    bool got1 = (evt & ICM1_DRDY_FLAG) != 0;
    bool got2 = (evt & ICM2_DRDY_FLAG) != 0;

    if(got1)
        ICM_TriggerRead(ICM_INSTANCE_1);
    if(got2)
        ICM_TriggerRead(ICM_INSTANCE_2);

    ICM_CopyTo(ICM_INSTANCE_1, &d1);
    ICM_CopyTo(ICM_INSTANCE_2, &d2);

    // 只有两路都拿到新数据时才交叉比对
    if(got1 && got2)
    {
        if(CrossCheck_IsAbnormal(&d1, &d2))
        {
            Health_RecordBad();
        }
        else
        {
            Health_RecordGood();
        }
    }
    else
    {
        Health_RecordGood();
    }

    *out = (g_imu_health.active_imu_sel == 0) ? d1 : d2;
    *dt_s = DWT_MeasureDt();
    return true;
}
