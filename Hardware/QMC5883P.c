#include "QMC5883P.h"
#include "ICM_42688P.h"
#include "SW_I2C.h"
#include "Delay.h"


#define QMC_ADDR    0x2C
#define QMC_Ready   1

QMC_Data_t qmc_data = {0};


/**
  * @brief  初始化QMC基本寄存器
  * @param	None
  * @retval None
  */
void QMC_Init(){
	Delay_ms(1);				// POR Complication Time --max 250us
	IIC_WriteReg(QMC_ADDR, 0x29, 0x06);
	IIC_WriteReg(QMC_ADDR, 0x0B, 0x08);
	IIC_WriteReg(QMC_ADDR, 0x0A, 0xC7);	// OSR2: 8; OSR1: 8; ODR: 50Hz; Continuous Mode
}


/**
  * @brief  QMC5883L 硬件偏置自测函数（阈值采集专用）
  * @note   [TODO/标定中] 当前仅用于采集 Set/Reset 脉冲前后的原始数据对，暂不参与起飞逻辑拦截。
  * 后续工程任务：需离线分析黑匣子记录的 (rmx1-rmx2) Delta 差值分布，
  * 找出合理的方差/极值边界后，再将此函数重构为返回布尔值的真实自检拦截器。
  * @param  f 指向黑匣子数据帧的指针，用于将两组原始磁场数据压入日志
  * @retval None
  */
void QMC_SelfTest(BB_Frame_t *f){
	uint8_t res;
	QMC_Data_t test = {0};
	int16_t rmx2 = 0, rmy2 = 0, rmz2 = 0;
	Delay_ms(1);				// POR Complication Time --max 250us
	
	IIC_WriteReg(QMC_ADDR, 0x29, 0x06);
	IIC_WriteReg(QMC_ADDR, 0x0A, 0x03);
	
	while(1){
		IIC_ReadReg(QMC_ADDR, 0x09, &res);
		if((res & 0x01) == QMC_Ready) break; 
	}
	
	QMC_ReadRaw_3Axis(&test);
	f->rmx1 = test.rmx; f->rmy1 = test.rmy; f->rmz1 = test.rmz;

	IIC_WriteReg(QMC_ADDR, 0x0B, 0x40);
	Delay_ms(5);
	
	QMC_ReadRaw_3Axis(&test);
	f->rmx2 = test.rmx; f->rmy2 = test.rmy; f->rmz2 = test.rmz;
	/* 做差 delta， 看看delta是否合理 */
}
/**
  * @brief  Burst读QMC3轴磁力计原始数据
  * @param	MX: X轴
  * @param	MY: Y轴
  * @param	MZ: Z轴  
  * @retval 状态码
  *		@arg 0: 读取正常
  *		@arg 1: 读取异常
  */
uint8_t QMC_ReadRaw_3Axis(QMC_Data_t *qmc_data){
	uint8_t buf[6] = {0};
	uint8_t ret = 1;
	
	IIC_Start();
	if(IIC_SendSlaveAddr(QMC_ADDR, Write) == NAck) goto stop;
	if(IIC_SendRegAddr(0x01) == NAck) goto stop;
	IIC_Start();
	if(IIC_SendSlaveAddr(QMC_ADDR, Read) == NAck) goto stop;
		
	for(int i=0;i<6;i++){
		IIC_ReadData(&buf[i]);
		if(i == 5) IIC_SendNAck();
		else IIC_SendAck();
	}
	ret = 0;
	
	qmc_data->rmx = (int16_t)(buf[1] << 8 | buf[0]);
	qmc_data->rmy = (int16_t)(buf[3] << 8 | buf[2]);
	qmc_data->rmz = (int16_t)(buf[5] << 8 | buf[4]);
stop:
	IIC_Stop();
	return ret;
}
/**
  * @brief  磁力计原始值转换为Gauss
  * @param  MX/MY/MZ  转换结果输出，单位Gauss
  * @param  RX/RY/RZ  磁力计三轴原始值，单位LSB
  * @retval None
  */
void QMC_Raw2Gauss(QMC_Data_t *qmc_data){
    qmc_data->MX = qmc_data->rmx * QMC_SENSITIVITY;
    qmc_data->MY = -1 * qmc_data->rmy * QMC_SENSITIVITY;		// 取负数，MY才符合 NED 的Y轴
    qmc_data->MZ = -1 * qmc_data->rmz * QMC_SENSITIVITY;		// 取负数，MZ才符合 NED 的Z轴
}


