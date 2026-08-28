/**
 * @file    bsp_icm42688p.c
 * @brief   双 ICM IMU 初始化，DRDY 通知及 SPI 数据读取实现。
 */

#include "bsp_icm42688.h"

/* ICM 寄存器地址。 */
#define REG_WHO_AM_I 0x01
#define REG_INT_CFG1 0x06

/*
 * Accel / Gyro 连续数据 Burst Read 起始地址。
 * 从该地址连续读取 12 Byte，可获得三轴  Accel 与三轴 Gyro。
 */
#define REG_ACC_XH 0x0C

#define REG_ACC_RANGE 0x41
#define REG_ACC_CONF 0x40
#define REG_GYR_RANGE 0x43
#define REG_GYR_CONF 0x42
#define REG_PWR_CTRL 0x7D
#define REG_SOFT_RST 0x4A

#define WHO_AM_I_VAL 0x6A   // WHO_AM_I 期望值

#define VAL_SOFT_RST 0xA5   // ICM 初始化配置值，复位整个电路，数值清零。
#define VAL_PWR_CTRL 0x0E   // 温度、Accel、Gyro 使能。
#define VAL_ACC_RANGE 0x03  // Accel Full Scale = ±16 g。
#define VAL_ACC_CONF 0x8B   // Accel Filter / Bandwidth / ODR 配置。
#define VAL_GYR_RANGE 0x00  // Gyro Full Scale = ±2000 deg/s。
#define VAL_GYR_CONF 0xCB   // Gyro Filter / Noise / Bandwidth / ODR 配置。
#define VAL_INT_CFG1 0x03   // INT1 配置为 Push-Pull、Active High，并输出 Gyro DRDY。

#define LSB_ACC 0.000488f   // Accel 原始 LSB 到 g 的比例系数，±16 g Range。
#define LSB_GYO 0.061f      // Gyro 原始 LSB 到 deg/s 的比例系数，±2000 deg/s Range。

/** 两颗 IMU 共用的 Data Ready Event Flags。 */
osEventFlagsId_t g_icmDataReadyEvtId = NULL;

/*
 * 单颗 ICM 的硬件绑定信息。
 * 
 * 每个实例分别关联自己的 SPI Bus 与 CS GPIO，
 * 上层通过 IcmInstance_t 访问，不直接关心具体硬件资源。
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} IcmHw_t;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

/** ICM 实例与底层 SPI / CS 硬件资源映射表。 */
static const IcmHw_t s_icmHw[ICM_INSTANCE_MAX] =
    {
        [ICM_INSTANCE_1] =
            {
                .hspi = &hspi1,
                .cs_port = GPIOC,
                .cs_pin = GPIO_PIN_4},

        [ICM_INSTANCE_2] =
            {
                .hspi = &hspi3,
                .cs_port = GPIOB,
                .cs_pin = GPIO_PIN_3},
};

/*
 * 每颗 ICM 最近一次成功读取并完成坐标转换的数据。
 * 
 * SPI 读取失败时保持上一帧有效数据，不使用无效 Rx Buffer 覆盖。
 */
static IcmData_t s_icmData[ICM_INSTANCE_MAX]; 

/*
 * 读取指定 ICM 的单个寄存器。
 * 
 * SPI Read Command 通过 Register Address bit7=1 表示读操作。
 * 第一个接收 Byte 对应命令阶段，因此实际寄存器值位于 rx[1]。
 */
static uint8_t ICM_ReadReg(IcmInstance_t inst, uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(
        s_icmHw[inst].hspi,
        tx,
        rx,
        2,
        10);

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_SET);

    return rx[1];
}

/*
 * 向指定 ICM 的单个寄存器写入数据。
 * 
 * SPI Write Command 将 Register Address bit7 清零。
 */
static void ICM_WriteReg(IcmInstance_t inst, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), data};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(
        s_icmHw[inst].hspi,
        tx,
        rx,
        2,
        10);

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_SET);
}

/*
 * 初始化指定 ICM 实例。
 * 
 * 初始化流程：
 * 
 * 1. Soft Reset
 * 2. WHO_AM_I Verification
 * 3. Power / Accel / Gyro Configuraion
 * 4. DRDY Interrupt Configuration
 * 
 * WHO_AM_I 不匹配时立即返回失败。
 */
static bool ICM_InitOne(IcmInstance_t inst)
{
    uint8_t who;

    ICM_WriteReg(inst, REG_SOFT_RST, VAL_SOFT_RST);
    osDelay(20);

    who = ICM_ReadReg(inst, REG_WHO_AM_I);
    if(who != WHO_AM_I_VAL){
        return false;
    }

    ICM_WriteReg(inst, REG_PWR_CTRL, VAL_PWR_CTRL);
    osDelay(20);

    ICM_WriteReg(inst, REG_ACC_RANGE, VAL_ACC_RANGE);
    osDelay(5);

    ICM_WriteReg(inst, REG_ACC_CONF, VAL_ACC_CONF);
    osDelay(5);

    ICM_WriteReg(inst, REG_GYR_RANGE, VAL_GYR_RANGE);
    osDelay(5);

    ICM_WriteReg(inst, REG_GYR_CONF, VAL_GYR_CONF);
    osDelay(10);

    ICM_WriteReg(inst, REG_INT_CFG1, VAL_INT_CFG1);
    osDelay(1);

    return true;
}

/*
 * 主动执行一次与正常采样相同范围的 SPI Burst Read，
 * 用数据寄存器读取动作清除上电阶段可能已经锁存的 DRDY 状态。
 * 
 * 如果 INT1 配置生效时内部 DRDY 已经为有效状态，
 * INT Pin 可能直接保持 High，而 EXTI 配置只检测 Rising Edge。
 * 此时不会再产生新的 Low -> High 跳变，第一次 DRDY IRQ 可能无法出现。
 * 
 * 初始化结束前主动读取一次数据寄存器，可清除该锁存状态，
 * 使 INT Pin 回到可产生下一次 Rising Edge 的状态。
 * 
 * 此次读取仅用于清除 DRDY，不解析返回的数据。
 */
static void ICM_ClearDataReadyLatch(IcmInstance_t inst)
{
    uint8_t tx[13] = {REG_ACC_XH | 0x80, 0};
    uint8_t rx[13] = {0};

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(
        s_icmHw[inst].hspi,
        tx,
        rx,
        13,
        10);

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_SET);
}

uint8_t ICM_InitAll(void)
{
    uint8_t fail_mask = 0;

    /*
     * DRDY ISR 只负责设置 Event Flag，
     * 实际 SPI 采样由等待这些 Flag 的 IMU Task 执行。
     */
    g_icmDataReadyEvtId = osEventFlagsNew(NULL);

    /* 初始化前确保两颗 IMU 的 CS 均处于未选中状态。 */
    HAL_GPIO_WritePin(s_icmHw[ICM_INSTANCE_1].cs_port, s_icmHw[ICM_INSTANCE_1].cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_icmHw[ICM_INSTANCE_2].cs_port, s_icmHw[ICM_INSTANCE_2].cs_pin, GPIO_PIN_SET);
	
    if(!ICM_InitOne(ICM_INSTANCE_1))
    {
        fail_mask |= (1U << 0);
    }

    if(!ICM_InitOne(ICM_INSTANCE_2))
    {
        fail_mask |=
            (1U << 1);
    }

    /*
     * 清除初始化过程中可能锁存的 DRDY，
     * 保证后续 EXTI 能检测真正的数据就绪 Rising Edge。
     */
    ICM_ClearDataReadyLatch(ICM_INSTANCE_1);
	ICM_ClearDataReadyLatch(ICM_INSTANCE_2);
	
    return fail_mask;
}

void ICM_OnDataReady_ISR(IcmInstance_t inst)
{
    /*
     * ISR 中不执行 SPI Transaction。
     * 
     * 这里只把“哪颗 IMU 有新数据”转换为 RTOS Event Flag，
     * 由上层 IMU Task 在 Task Context 中执行真正的数据读取。
     */
    const uint32_t flag =
        (inst == ICM_INSTANCE_1)
            ? ICM1_DRDY_FLAG
            : ICM2_DRDY_FLAG;

    osEventFlagsSet(g_icmDataReadyEvtId, flag);
}

void ICM_TriggerRead(IcmInstance_t inst)
{
    /*
     * 1 Byte Read Command + 12 Byte Sensor Data。
     * 
     * rx[0] 对应 SPI Command 阶段，
     * rx[1] ~ rx[12] 为三轴 Accel + 三轴 Gyro 原始数据。
     */
    uint8_t tx[13] = {REG_ACC_XH | 0x80, 0};
    uint8_t rx[13] = {0};

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_RESET);

    const HAL_StatusTypeDef ret =
        HAL_SPI_TransmitReceive(
            s_icmHw[inst].hspi,
            tx,
            rx,
            13,
            10);

    HAL_GPIO_WritePin(
        s_icmHw[inst].cs_port,
        s_icmHw[inst].cs_pin,
        GPIO_PIN_SET);

    /*
     * SPI 失败时不更新 s_icmDate[]。
     * 
     * 上层仍然保留上一帧有效数据，
     * 避免用本次无效 Rx Buffer 覆盖有效 Sensor State。
     */
    if (ret != HAL_OK)
	{
		return;
	}

    /* 将每轴两个 Byte 合成为有符号 16-bit Raw Sample。 */
    int16_t raw_ax = (int16_t)((rx[1] << 8) | rx[2]);
    int16_t raw_ay = (int16_t)((rx[3] << 8) | rx[4]);
    int16_t raw_az = (int16_t)((rx[5] << 8) | rx[6]);
    int16_t raw_gx = (int16_t)((rx[7] << 8) | rx[8]);
    int16_t raw_gy = (int16_t)((rx[9] << 8) | rx[10]);
    int16_t raw_gz = (int16_t)((rx[11] << 8) | rx[12]);

    /*
     * 根据 PCB 上 IMU 的实际安装方向，
     * 将 Sensor Frame 转换为飞控统一使用的 Body / NED Axis Convention。
     * 
     * Accel：X/Y 互换；
     * Gyro：X/Y 互换并取反；
     * Z Axis 保持当前方向。
     */
    s_icmData[inst].ax = raw_ay * LSB_ACC;
    s_icmData[inst].ay = raw_ax * LSB_ACC;
    s_icmData[inst].az = raw_az * LSB_ACC;
    s_icmData[inst].gx = -(raw_gy * LSB_GYO);
    s_icmData[inst].gy = -(raw_gx * LSB_GYO);
    s_icmData[inst].gz = raw_gz * LSB_GYO;

    /*
     * 保存本次有效 Sample 的 Cycle Timestamp，
     * 供上层计算 Sample 间隔，Freshness 或双 IMU 时间关系。
     */
    s_icmData[inst].timestamp_cycle = DWT->CYCCNT;
}

void ICM_CopyTo(IcmInstance_t inst, IcmData_t *out)
{
    if(out == NULL || inst >= ICM_INSTANCE_MAX)
        return;
    *out = s_icmData[inst];
}
