#ifndef __TFCARD_H
#define __TFCARD_H

#include "stm32f4xx.h"

#define SD_CS_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define SD_CS_HIGH() do{ \
	uint8_t _r; \
	GPIO_SetBits(GPIOB, GPIO_Pin_12); \
	SD_SPI_RWByte(&_r, 0xFF); \
}while(0)
//#define SD_CS_HIGH()	GPIO_SetBits(GPIOB, GPIO_Pin_12)

void SD_SPI_Init(void);
void SD_SPI_SetSpeed(uint16_t prescaler);
void SD_SPI_RWByte(uint8_t *RXD, uint8_t TXD);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg);
int8_t SD_Init(void);

int8_t SD_ReadBlock(uint32_t block, uint8_t *buf);
int8_t SD_WriteBlock(uint32_t block,  const uint8_t *buf);

#endif