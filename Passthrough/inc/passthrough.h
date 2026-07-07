#ifndef __PASSTHROUGH_H
#define __PASSTHROUGH_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*============================================================================
 * 4-Way Interface Passthrough for BLHeli ESC
 * 从 Betaflight 移植，适配 STM32F405 + SPL
 * 用途：让 esc-configurator.com 通过 CH340X USB 串口刷写 ESC 固件
 *
 * 硬件连接：
 *   USART3 (PB10 TX / PB11 RX) - CH340X USB 串口，连 PC
 *   PA8  - ESC S1 信号线 (TIM1_CH1)
 *   PA9  - ESC S2 信号线 (TIM1_CH2)
 *   PA10 - ESC S3 信号线 (TIM1_CH3)
 *   PA11 - ESC S4 信号线 (TIM1_CH4)
 *
 * 注意：此固件为临时用途，刷完 ESC 后换回正常飞控固件
 *============================================================================*/

/* ---- ESC 信号引脚定义 ---- */
#define ESC_COUNT       4

#define ESC1_PIN        GPIO_Pin_8
#define ESC1_PORT       GPIOA
#define ESC2_PIN        GPIO_Pin_9
#define ESC2_PORT       GPIOA
#define ESC3_PIN        GPIO_Pin_10
#define ESC3_PORT       GPIOA
#define ESC4_PIN        GPIO_Pin_11
#define ESC4_PORT       GPIOA

/* ---- MSP 协议 ---- */
#define MSP_API_VERSION         1
#define MSP_FC_VARIANT          2
#define MSP_FC_VERSION          3
#define MSP_BOARD_INFO          4
#define MSP_BUILD_INFO          5
#define MSP_FEATURE_CONFIG      36
#define MSP_MOTOR               104
#define MSP_MOTOR_CONFIG        131
#define MSP_UID                 160
#define MSP_SET_MOTOR           214
#define MSP_SET_PASSTHROUGH     245

/* MSP 帧格式: $M< len cmd [payload] crc  (请求)
 *             $M> len cmd [payload] crc  (响应)  */

/* ---- 4-Way Interface 命令 ---- */
#define cmd_Remote_Escape       0x2E
#define cmd_Local_Escape        0x2F

#define cmd_InterfaceTestAlive  0x30
#define cmd_ProtocolGetVersion  0x31
#define cmd_InterfaceGetName    0x32
#define cmd_InterfaceGetVersion 0x33
#define cmd_InterfaceExit       0x34
#define cmd_DeviceReset         0x35
#define cmd_DeviceInitFlash     0x37
#define cmd_DeviceEraseAll      0x38
#define cmd_DevicePageErase     0x39
#define cmd_DeviceRead          0x3A
#define cmd_DeviceWrite         0x3B
#define cmd_DeviceC2CK_LOW      0x3C
#define cmd_DeviceReadEEprom    0x3D
#define cmd_DeviceWriteEEprom   0x3E
#define cmd_InterfaceSetMode    0x3F
#define cmd_DeviceVerify        0x40

/* ---- 4-Way 响应码 ---- */
#define ACK_OK                  0x00
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
#define ACK_I_INVALID_CHANNEL   0x08
#define ACK_I_INVALID_PARAM     0x09
#define ACK_D_GENERAL_ERROR     0x0F

/* ---- 4-Way 接口模式 ---- */
#define imC2        0
#define imSIL_BLB   1
#define imATM_BLB   2
#define imSK        3
#define imARM_BLB   4

/* ---- 4-Way 版本 ---- */
#define SERIAL_4WAY_VER_MAIN    20
#define SERIAL_4WAY_VER_SUB_1   0
#define SERIAL_4WAY_VER_SUB_2   6
#define SERIAL_4WAY_PROTOCOL_VER 108
#define SERIAL_4WAY_VERSION     ((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)
#define SERIAL_4WAY_VERSION_HI  (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO  (uint8_t)(SERIAL_4WAY_VERSION % 100)

/* ---- BLHeli Bootloader 命令 ---- */
#define BL_CMD_RUN              0x00
#define BL_CMD_PROG_FLASH       0x01
#define BL_CMD_ERASE_FLASH      0x02
#define BL_CMD_READ_FLASH_SIL   0x03
#define BL_CMD_VERIFY_FLASH     0x03
#define BL_CMD_VERIFY_FLASH_ARM 0x04
#define BL_CMD_READ_EEPROM      0x04
#define BL_CMD_PROG_EEPROM      0x05
#define BL_CMD_READ_FLASH_ATM   0x07
#define BL_CMD_KEEP_ALIVE       0xFD
#define BL_CMD_SET_ADDRESS      0xFF
#define BL_CMD_SET_BUFFER       0xFE

/* Bootloader 响应 */
#define brSUCCESS               0x30
#define brERRORVERIFY           0xC0
#define brERRORCOMMAND          0xC1
#define brERRORCRC              0xC2
#define brNONE                  0xFF

/* Bit-bang 时序 (19200 baud) */
#define BIT_TIME_US             52
#define BIT_TIME_HALF_US        26
#define BIT_TIME_3_4_US         39
#define START_BIT_TIMEOUT_MS    2

/* ---- IO memory 结构 ---- */
typedef struct {
    uint8_t D_NUM_BYTES;
    uint8_t D_FLASH_ADDR_H;
    uint8_t D_FLASH_ADDR_L;
    uint8_t *D_PTR_I;
} ioMem_t;

typedef union __attribute__((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

typedef union __attribute__((packed)) {
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
} uint8_32_u;

/* ---- 函数声明 ---- */

/* 硬件初始化 */
void Passthrough_Init(void);

/* 主循环：MSP 解析 + 4-way interface */
void Passthrough_Run(void);

/* 微秒精确计时 (DWT) */
void DWT_Init(void);
uint32_t DWT_GetMicros(void);

#endif /* __PASSTHROUGH_H */