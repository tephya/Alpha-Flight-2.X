#include "bsp_qmc5883.h"


#define IIC_CHECK(x) do{                \
            HAL_StatusTypeDef _s = (x); \
            if(_s != HAL_OK)            \
                return _s;              \
        }while(0)

#define QMC_SENSITIVITY (1.0f / 3750.0f)
#define QMC_ADDR 0x2C
#define QMC_REG_STATUS 0x09
#define QMC_STATUS_OVFL 0x02

typedef struct
{
    int16_t rmx, rmy, rmz;
    float MX, MY, MZ;
    bool ovfl; // true=任一一轴数据溢出(超过±30000 LSB)，本次数据不可信
} MAG_Data_t;

static MAG_Data_t mag_data;

/**
 * @brief   初始化QMC寄存器
 * @retval  HAL_OK=成功，其余为HAL错误码
 */
HAL_StatusTypeDef QMC_Init(void)
{
    HAL_Delay(1); // POR Complication Time --max 250us

    IIC_CHECK(IIC_WriteReg(QMC_ADDR, 0x29, 0x06));
    IIC_CHECK(IIC_WriteReg(QMC_ADDR, 0x0B, 0x08));  // ±8g
    IIC_CHECK(IIC_WriteReg(QMC_ADDR, 0x0A, 0xC7));  // OSR2: 8; OSR1: 8; ODR: 50Hz; Continuous Mode

    return HAL_OK;
}

/**
 * @brief   原始数据转物理单位，并对齐NED坐标系
 */
static void QMC_Raw2Gauss(void)
{
    mag_data.MX = mag_data.rmx * QMC_SENSITIVITY;
    mag_data.MY = -1 * mag_data.rmy * QMC_SENSITIVITY;  // 取负数，MY才符合 NED 的Y轴(符合作者的机体坐标系，请根据实际情况来)
    mag_data.MZ = -1 * mag_data.rmz * QMC_SENSITIVITY;  // 取负数，MZ才符合 NED 的Z轴
}

/**
 * @brief   读磁力计数据，内部完成一次数据数据读取+转换，结果静态存入mag_data
 * @retval  HAL_OK=OK，其余为HAL错误码
 */
HAL_StatusTypeDef QMC_ReadData(void)
{
    uint8_t status = 0;
    uint8_t buf[6] = {0};

    // 先读状态寄存器拿OVFL，读取会自动清位，所以这次读到的就是“上一帧结果”
    IIC_CHECK(IIC_ReadReg(QMC_ADDR, QMC_REG_STATUS, &status));
    mag_data.ovfl = (status & QMC_STATUS_OVFL) != 0;

    IIC_CHECK(IIC_ReadBurst(QMC_ADDR, 0x01, buf, 6));

    mag_data.rmx = (int16_t)(buf[1] << 8 | buf[0]);
    mag_data.rmy = (int16_t)(buf[3] << 8 | buf[2]);
    mag_data.rmz = (int16_t)(buf[5] << 8 | buf[4]);

    QMC_Raw2Gauss();

    return HAL_OK;
}

/**
 * @brief   把最近读到的数据拷贝一份给调用方
 * @param   out 调用方提供的接收结构体指针
 */
void QMC_CopyTo(MagData_t *out)
{
    out->MX = mag_data.MX;
    out->MY = mag_data.MY;
    out->MZ = mag_data.MZ;
    out->ovfl = mag_data.ovfl;
}
