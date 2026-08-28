/**
 * @file    bsp_qmc5883.c
 * @brief   QMC 磁力计寄存器配置，数据读取及坐标转换实现。
 */

#include "bsp_qmc5883.h"
#include "cmsis_os2.h"

/*
 * 执行一次 I2C 操作并检查 HAL 返回值。
 * 
 * 任何一步通信失败时立即结束当前函数，
 * 将对应 HAL Error Code 返回给调用方。
 */
#define IIC_CHECK(x) do{                \
            HAL_StatusTypeDef _s = (x); \
            if(_s != HAL_OK)            \
                return _s;              \
        }while(0)

#define QMC_SENSITIVITY (1.0f / 3750.0f)    // Raw LSB 转 Gauss 的比例系数。
#define QMC_ADDR 0x2C           // QMC 7-bit I2C Device Address。
#define QMC_REG_STATUS 0x09     // Status Register 地址。

/* QMC 寄存器地址。 */
#define QMC_REG_DATA_X_L 0x01
#define QMC_REG_STATUS 0x09
#define QMC_REG_CONTROL_1 0x0A
#define QMC_REG_CONTROL_2 0x0B
#define QMC_REG_SET_RESET 0x29

/* 
 * QMC 状态位。
 * Status Register 中的 Overflow Flag。
 */
#define QMC_STATUS_OVFL 0x02

/* QMC 初始化配置值。 */
#define QMC_VAL_SET_RESET 0x06
#define QMC_VAL_CONTROL_2 0x08
#define QMC_VAL_CONTROL_1 0xC7

/*
 * 驱动内部磁力计数据。
 * 
 * 同时保存 Sensor Raw Data 与经过 Scale / Axis Mapping 后的物理量，
 * 对外仅暴露转换后的 MagData_ts。
 */
typedef struct
{
    int16_t rmx;
    int16_t rmy;
    int16_t rmz;

    float MX;
    float MY;
    float MZ;

    bool ovfl;
} MAG_Data_t;

static MAG_Data_t mag_data;     // 最近一次成功读取的磁力计数据。

/*
 * 将磁力计 Raw Data 转换为 Gauss，
 * 并根据传感器实际安装方向映射到机体 NED 坐标系。
 * 
 * 当前安装关系：
 * 
 * Sensor +X -> Body -X
 * Sensor +Y -> Body +Y
 * Sensor +Z -> Body -Z
 */
static void QMC_Raw2Gauss(void)
{
    mag_data.MX = -mag_data.rmx * QMC_SENSITIVITY;
    mag_data.MY = mag_data.rmy * QMC_SENSITIVITY;
    mag_data.MZ = -mag_data.rmz * QMC_SENSITIVITY;
}

HAL_StatusTypeDef QMC_Init(void)
{
    /*
     * 等待 Power-On Reset 完成。
     * 当前延时明显大于器件要求的最大 POR Completion Time。
     */
    osDelay(1);

    IIC_CHECK(
        IIC_WriteReg(
            QMC_ADDR,
            QMC_REG_SET_RESET,
            QMC_VAL_SET_RESET));

    /* 配置 Measurement Range。 */
    IIC_CHECK(
        IIC_WriteReg(
            QMC_ADDR,
            QMC_REG_CONTROL_2,
            QMC_VAL_CONTROL_2));

    /*
     * 配置 Oversampling、ODR 及 Continuous Measurement Mode。
     *
     * OSR2 = 8
     * OSR1 = 8
     * ODR  = 50 Hz
     */
    IIC_CHECK(
        IIC_WriteReg(
            QMC_ADDR,
            QMC_REG_CONTROL_1,
            QMC_VAL_CONTROL_1));

    return HAL_OK;
}

HAL_StatusTypeDef QMC_ReadData(void)
{
    uint8_t status = 0;
    uint8_t buf[6] = {0};

    /*
     * 先读取 Status Register 获取 Overflow 状态。
     * 
     * 该状态在读取后会被器件清除，
     * 因此这里保存的是与当前待读取 Sensor Data 对应的状态信息。
     */
    IIC_CHECK(IIC_ReadReg(QMC_ADDR, QMC_REG_STATUS, &status));
    mag_data.ovfl = (status & QMC_STATUS_OVFL) != 0;

    /*
     * 从三轴数据起始寄存器连续读取 6 Byte：
     *
     * X_L, X_H,
     * Y_L, Y_H,
     * Z_L, Z_H。
     */
    IIC_CHECK(IIC_ReadBurst(QMC_ADDR, QMC_REG_DATA_X_L, buf, 6));

    mag_data.rmx = (int16_t)(buf[1] << 8 | buf[0]);
    mag_data.rmy = (int16_t)(buf[3] << 8 | buf[2]);
    mag_data.rmz = (int16_t)(buf[5] << 8 | buf[4]);

    QMC_Raw2Gauss();

    return HAL_OK;
}

void QMC_CopyTo(MagData_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->MX = mag_data.MX;
    out->MY = mag_data.MY;
    out->MZ = mag_data.MZ;
    out->ovfl = mag_data.ovfl;
}
