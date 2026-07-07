#ifndef __SW_I2C_H
#define __SW_I2C_H

#include "stm32f4xx.h"

#define SCL_VAL		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)
#define SDA_VAL		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)

/* Set: 拉高 （释放） 的意思；
 * open-drain 模式下会反转内部逻辑电平到 MOSFET 的 Gate，导致SetBit，
 * 实际传到 Gate 的电平是低，MOSFET 不导通；
 * 之所以用 open-drain，是为了迎合 IIC 开漏输出这一特效的设定
 * 
 * 补充： 从机不会控制SCL，从机只能控制SDA
 */
#define Set_SCL		(GPIO_SetBits(GPIOB, GPIO_Pin_6))		
#define Reset_SCL	(GPIO_ResetBits(GPIOB, GPIO_Pin_6))
#define Set_SDA		(GPIO_SetBits(GPIOB, GPIO_Pin_7))
#define Reset_SDA	(GPIO_ResetBits(GPIOB, GPIO_Pin_7))

#define Ack		1
#define NAck	0

#define Write	0
#define Read	1

void IIC_Init();

uint8_t IIC_WriteReg(uint8_t addr, uint8_t Reg, uint8_t Data);
uint8_t IIC_ReadReg(uint8_t addr, uint8_t Reg, uint8_t *Data);
uint8_t IIC_SendSlaveAddr(uint8_t addr, uint8_t rw); 
uint8_t IIC_SendRegAddr(uint8_t reg); 

void IIC_Start();
void IIC_Stop();
uint8_t IIC_WaitAck();
void IIC_SendNAck();
void IIC_SendAck();

void IIC_SendData(uint8_t Data);
void IIC_ReadData(uint8_t *Data);

#endif
