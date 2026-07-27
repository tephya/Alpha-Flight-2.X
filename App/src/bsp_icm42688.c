#include "bsp_icm42688.h"

/*====== 寄存器地址 ======*/
#define REG_WHO_AM_I 0x01
#define REG_INT_CFG1 0x06
#define REG_ACC_XH 0x0C     // burst读起点，连续12字节到GYR_ZL(0x17)
#define REG_ACC_RANGE 0x41
#define REG_ACC_CONF 0x40
#define REG_GYR_RANGE 0x43
#define REG_GYR_CONF 0x42
#define REG_PWR_CTRL 0x7D
#define REG_SOFT_RST 0x4A

#define WHO_AM_I_VAL 0x6A

/*====== 寄存器配置值(ODR=800Hz) ====== */
#define VAL_SOFT_RST 0xA5
#define VAL_PWR_CTRL 0x0E   // TEMP_EN + ACC_EN + GYR_EN
#define VAL_ACC_RANGE 0x03  // ±16G
#define VAL_ACC_CONF 0x8B   // FILTER_PERF=1, BWP=000, ODR=800Hz
#define VAL_GYR_RANGE 0x00  // ±2000dps
#define VAL_GYR_CONF 0xCB   // FILTER_PERF=1, NOISE_PERF=1, BWP=00, ODR=800Hz
#define VAL_INT_CFG1 0x03   // 推挽输出，高有效，DRDY_GYR

#define LSB_ACC 0.000488f   // ±16G, 32768 LSB
#define LSB_GYO 0.061f      // ±2000dps, 32768 LSB

osEventFlagsId_t g_icmDataReadyEvtId = NULL;

// 每个实例的硬件描述(CS/SPI句柄)
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} IcmHw_t;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

static const IcmHw_t s_icmHw[ICM_INSTANCE_MAX] =
{
    [ICM_INSTANCE_1] = {.hspi = &hspi1, .cs_port = GPIOC, .cs_pin = GPIO_PIN_4},
    [ICM_INSTANCE_2] = {.hspi = &hspi3, .cs_port = GPIOB, .cs_pin = GPIO_PIN_3},
};

static IcmData_t s_icmData[ICM_INSTANCE_MAX];       // 内部static缓冲


/**
 * @brief   向指定ICM实体读指定地址的内容
 * @param   inst    指向的实体编号
 * @param   reg     指定的寄存器地址
 * @retval  寄存器的数据
 */
static uint8_t ICM_ReadReg(IcmInstance_t inst, uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(s_icmHw[inst].hspi, tx, rx, 2, 10);
    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_SET);

    return rx[1];
}

/**
 * @brief   向指定ICM实体的指定地址写数据
 * @param   inst    指向的实体编号
 * @param   reg     指定的寄存器地址
 * @param   data    要写入的数据
 */
static void ICM_WriteReg(IcmInstance_t inst, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), data};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(s_icmHw[inst].hspi, tx, rx, 2, 10);
    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_SET);
}

/**
 * @brief   初始化指定ICM实体
 * @param   inst    指定的实体编号
 * @retval  true=初始化成功, false=WHO_AM_I校验失败
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

/**
 * @brief   打破上电锁存死锁。发起一次跟ICM_TriggerRead同样窗口的burst SPI事务
 * @note    利用读写寄存器会清除DRDY锁存的作用，不解析这次读到的内容
 * @param   inst    ICM实体编号
 */
static void ICM_ClearDataReadyLatch(IcmInstance_t inst)
{
//	(void)ICM_ReadReg(inst, 0x0B);

    uint8_t tx[13] = {REG_ACC_XH | 0x80, 0};
    uint8_t rx[13] = {0};

    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(s_icmHw[inst].hspi, tx, rx, 13, 10);
    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_SET);
}

/**
 * @brief   初始化所有ICM实体
 * @retval  0=所有实体初始化成功,
 *          1=ICM实体2初始化成功,
 *          2=ICM实体1初始化成功,
 *          3=所有实体初始化均失败
 */
uint8_t ICM_InitAll(void)
{
    uint8_t fail_mask = 0;

    g_icmDataReadyEvtId = osEventFlagsNew(NULL);

    HAL_GPIO_WritePin(s_icmHw[ICM_INSTANCE_1].cs_port, s_icmHw[ICM_INSTANCE_1].cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_icmHw[ICM_INSTANCE_2].cs_port, s_icmHw[ICM_INSTANCE_2].cs_pin, GPIO_PIN_SET);
	
    if(!ICM_InitOne(ICM_INSTANCE_1))
        fail_mask |= (1U << 0);
    if(!ICM_InitOne(ICM_INSTANCE_2))
        fail_mask |= (1U << 1);

    /* 打破“上电即锁存死锁”：INT_CFG1配置生效瞬间，如果内部DRDY状态位恰好已经是1，
     * INT1会被直接顶到高电平所存住，EXTI只认Rising Edge，永远等不到“由低到高”的下一次跳变，
     * 第一次中断永远不会发生。而读取数据寄存器这个动作本身会清除锁存，
     * 所以这里手动强制读一次，只是借“读”这个动作的硬副作用把两颗IMU的INT1都先拉回低电平，
     * 让后续EXTI能从这里开始正常检测到每一次真正的Rising Edge*/	
    ICM_ClearDataReadyLatch(ICM_INSTANCE_1);
	ICM_ClearDataReadyLatch(ICM_INSTANCE_2);
	
    return fail_mask;
}

/**
 * @brief   ISR入口，由bsp_gpio_dispatch.c按Pin分流调用
 * @note    IMU1 INT1 = PA4, IMU2 INT1 = PA15;
 *          在HAL_GPIO_EXIT_Callback的dispatch里加：
 *              case GPIO_PIN_4: ICM_OnDataReady_ISR(ICM_INSTANCE_1); break;
 *              case GPIO_PIN_15: ICM_OnDataReady_ISR(ICM_INSTANCE_2); break;
 * @param   inst    指定的ICM实体编号
 */
void ICM_OnDataReady_ISR(IcmInstance_t inst)
{
    uint32_t flag = (inst == ICM_INSTANCE_1) ? ICM1_DRDY_FLAG : ICM2_DRDY_FLAG;
    osEventFlagsSet(g_icmDataReadyEvtId, flag);
}

/**
 * @brief   对指定的ICM实体进行数据读取+转换
 * @param   inst    指定的数据实体编号
 */
void ICM_TriggerRead(IcmInstance_t inst)
{
    uint8_t tx[13] = {REG_ACC_XH | 0x80, 0};
    uint8_t rx[13] = {0};

    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(s_icmHw[inst].hspi, tx, rx, 13, 10);
    HAL_GPIO_WritePin(s_icmHw[inst].cs_port, s_icmHw[inst].cs_pin, GPIO_PIN_SET);

	if (ret != HAL_OK)
	{
		return;   // 传输失败，直接放弃这次转换，保留上一次的旧数据，不要用垃圾值覆盖
	}

    int16_t raw_ax = (int16_t)((rx[1] << 8) | rx[2]);
    int16_t raw_ay = (int16_t)((rx[3] << 8) | rx[4]);
    int16_t raw_az = (int16_t)((rx[5] << 8) | rx[6]);
    int16_t raw_gx = (int16_t)((rx[7] << 8) | rx[8]);
    int16_t raw_gy = (int16_t)((rx[9] << 8) | rx[10]);
    int16_t raw_gz = (int16_t)((rx[11] << 8) | rx[12]);

    // ax/ay, gx/gy互换，贴合作者安装的IMU的NED轴
    s_icmData[inst].ax = raw_ay * LSB_ACC;
    s_icmData[inst].ay = raw_ax * LSB_ACC;
    s_icmData[inst].az = raw_az * LSB_ACC;
    s_icmData[inst].gx = raw_gy * LSB_GYO;
    s_icmData[inst].gy = raw_gx * LSB_GYO;
    s_icmData[inst].gz = raw_gz * LSB_GYO;
    s_icmData[inst].timestamp_cycle = DWT->CYCCNT;
}

/**
 * @brief   把指定ICM实体最近记录的数据拷贝一份给调用方
 * @param   inst    指定实体的编号
 * @param   out     调用方提供的接收实体指针
 */
void ICM_CopyTo(IcmInstance_t inst, IcmData_t *out)
{
    if(out == NULL || inst >= ICM_INSTANCE_MAX)
        return;
    *out = s_icmData[inst];
}
