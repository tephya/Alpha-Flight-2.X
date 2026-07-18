#ifndef __ELRS_H
#define __ELRS_H

#include "stm32f4xx.h"


typedef struct{
	uint16_t channels[16];			// 16 通道原始值 （172~1811）
	uint8_t	 rssi;					// 链路RSSI
	uint8_t  lq;					// 链路质量 （0~100）
	int8_t	 snr;					// 信噪比
	volatile uint8_t updated;		// 解析器置1， 消费者读完清0
} CRSF_Data_t;

extern CRSF_Data_t crsf_data, temp_data;

void ELRS_Init(void);
void ELRS_Poll(void);
void ELRS_StateMachine(uint8_t byte);
uint8_t CRSF_CRC8(void);
int8_t CRSF_PARSE_FRAME(void);



#endif
