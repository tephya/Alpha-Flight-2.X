#ifndef __ICM_42688P_H
#define __ICM_42688P_H

#ifndef M_PI
#define M_PI 		3.14159265358979323846f
#endif

#include "stm32f4xx.h"

#define	LSB_ACC		0.000488f
#define LSB_GYO		0.061f
#define GYRO_TRUST	0.998f

typedef struct {
	float ax, ay, az;
	float gx, gy, gz;
} IMU_Data_t;

extern IMU_Data_t imu1_data, imu2_data;


void ICM_SPI_Init(void);
void ICM_TIM7_Trigger_Init(void);
void ICM_SPI1_DMA_Init(void);
void ICM_Start_DMA_Transfer(void);
void DMA2_Stream0_IRQHandler(void);
uint8_t ICM_Init();

void ICM_SPI_RWByte(SPI_TypeDef * SPIx, uint8_t *RXData, uint8_t TXData);
uint8_t ICM_ReadReg(SPI_TypeDef * SPIx, uint8_t reg);
void ICM_WriteReg(SPI_TypeDef * SPIx, uint8_t reg, uint8_t Data);

void ICM_ReadACCData(SPI_TypeDef * SPIx, IMU_Data_t *imu_data);

#endif
