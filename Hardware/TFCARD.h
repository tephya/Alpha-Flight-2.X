#ifndef __TFCARD_H
#define __TFCARD_H

#include "stm32f4xx.h"

void SD_SPI_Init(void);
int8_t SD_Init(void);
void SD_SPI_DMA_Init(void);

static int8_t SD_DMA_WaitComplete(uint32_t timeout_ms);
int8_t SD_SPI_DMA_Transmit(const uint8_t *buf, uint16_t len);

void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);

void SD_SPI_SetSpeed(uint16_t prescaler);
void SD_SPI_RWByte(uint8_t *RXD, uint8_t TXD);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg);

int8_t SD_ReadBlock(uint32_t block, uint8_t *buf);
int8_t SD_WriteBlock(uint32_t block,  const uint8_t *buf);

#endif