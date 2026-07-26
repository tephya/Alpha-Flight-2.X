#ifndef __BSP_DSHOT_H
#define __BSP_DSHOT_H

#include <stdint.h>

void BSP_DSHOT_Init(void);
void BSP_DSHOT_Send(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4);

#endif
