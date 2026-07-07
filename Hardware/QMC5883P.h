#ifndef __QMC5883P_H
#define __QMC5883P_H

#include "stm32f4xx.h"
#include "blackbox.h"


#define QMC_SENSITIVITY		(1.0f / 3750.0f)
#define M_PI 3.14159265358979323846f


typedef struct {
	int16_t rmx, rmy, rmz;
	float MX, MY, MZ;
} QMC_Data_t;

extern QMC_Data_t qmc_data;

void QMC_Init(void);
void QMC_SelfTest(BB_Frame_t *f);
uint8_t QMC_ReadRaw_3Axis(QMC_Data_t *qmc_data);
void QMC_Raw2Gauss(QMC_Data_t *qmc_data);
					


#endif
