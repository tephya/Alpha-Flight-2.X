#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H

#include "bsp_icm42688.h"

void DebugUart_PrintImuDiff(const IcmData_t *d1, const IcmData_t *d2);

#endif
