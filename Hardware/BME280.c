#include "BME280.h"
#include "SW_I2C.h"

#define BME_ADDR	0x76

uint8_t BME280_ReadReg(uint8_t reg)
{
	uint8_t res;
	IIC_ReadReg(BME_ADDR, reg, &res);
	return res;
}