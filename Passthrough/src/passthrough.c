/*============================================================================
 * 4-Way Interface Passthrough - 从 Betaflight 移植
 * 目标: STM32F405RGT6, SPL, Keil MDK
 *============================================================================*/

#include "passthrough.h"

/* ========== 全局状态 ========== */

static uint8_t selected_esc = 0;
static uint8_32_u DeviceInfo;
static uint8_t CurrentInterfaceMode = imSIL_BLB;

/* ESC 引脚表 */
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} escPins[ESC_COUNT] = {
    { ESC1_PORT, ESC1_PIN },  /* PA8  - S1 */
    { ESC2_PORT, ESC2_PIN },  /* PA9  - S2 */
    { ESC3_PORT, ESC3_PIN },  /* PA10 - S3 */
    { ESC4_PORT, ESC4_PIN },  /* PA11 - S4 */
};

/* ========== DWT 微秒计时 ========== */

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetMicros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

static uint32_t DWT_GetMillis(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000);
}

static void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ========== USART3 底层 (CH340X) ========== */

static void USART3_Init_115200(void)
{
    USART_InitTypeDef uartInit;
    GPIO_InitTypeDef gpioInit;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB10 = TX, PB11 = RX */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    gpioInit.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpioInit.GPIO_Mode  = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpioInit);

    /* 115200 8N1 - esc-configurator 默认波特率 */
    uartInit.USART_BaudRate             = 115200;
    uartInit.USART_WordLength           = USART_WordLength_8b;
    uartInit.USART_StopBits             = USART_StopBits_1;
    uartInit.USART_Parity               = USART_Parity_No;
    uartInit.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
    uartInit.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &uartInit);
    USART_Cmd(USART3, ENABLE);
}

static bool UART_RxReady(void)
{
    return (USART3->SR & USART_FLAG_RXNE) ? true : false;
}

static uint8_t UART_ReadByte(void)
{
    while (!(USART3->SR & USART_FLAG_RXNE));
    return (uint8_t)(USART3->DR & 0xFF);
}

static bool UART_ReadByteTimeout(uint8_t *data, uint32_t timeoutUs)
{
    uint32_t start = DWT_GetMicros();
    while (!(USART3->SR & USART_FLAG_RXNE)) {
        if (timeoutUs && ((DWT_GetMicros() - start) > timeoutUs)) {
            return false;  /* 超时 */
        }
    }
    *data = (uint8_t)(USART3->DR & 0xFF);
    return true;
}

static void UART_WriteByte(uint8_t b)
{
    while (!(USART3->SR & USART_FLAG_TXE));
    USART3->DR = b;
}

static void UART_Flush(void)
{
    while (!(USART3->SR & USART_FLAG_TC));
}

/* ========== ESC 信号引脚 GPIO 操作 ========== */

static void ESC_PinsInit(void)
{
    GPIO_InitTypeDef gpioInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* 先把 TIM1 关掉，释放引脚 */
    TIM_Cmd(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

    /* 配置为输入上拉 (默认高) */
    gpioInit.GPIO_Mode  = GPIO_Mode_IN;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

    for (int i = 0; i < ESC_COUNT; i++) {
        gpioInit.GPIO_Pin = escPins[i].pin;
        GPIO_Init(escPins[i].port, &gpioInit);
    }
}

static inline bool ESC_IsHi(uint8_t idx)
{
    return GPIO_ReadInputDataBit(escPins[idx].port, escPins[idx].pin) != Bit_RESET;
}

static inline bool ESC_IsLo(uint8_t idx)
{
    return GPIO_ReadInputDataBit(escPins[idx].port, escPins[idx].pin) == Bit_RESET;
}

static inline void ESC_SetHi(uint8_t idx)
{
    GPIO_SetBits(escPins[idx].port, escPins[idx].pin);
}

static inline void ESC_SetLo(uint8_t idx)
{
    GPIO_ResetBits(escPins[idx].port, escPins[idx].pin);
}

static void ESC_SetInput(uint8_t idx)
{
    GPIO_InitTypeDef gpioInit;
    gpioInit.GPIO_Pin   = escPins[idx].pin;
    gpioInit.GPIO_Mode  = GPIO_Mode_IN;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(escPins[idx].port, &gpioInit);
}

static void ESC_SetOutput(uint8_t idx)
{
    GPIO_InitTypeDef gpioInit;
    gpioInit.GPIO_Pin   = escPins[idx].pin;
    gpioInit.GPIO_Mode  = GPIO_Mode_OUT;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(escPins[idx].port, &gpioInit);
}

/* ========== DSHOT600 输出 (电机控制) ========== */

#define DSHOT_ARR       279     /* 168MHz / 280 = 600kHz = DSHOT600 */
#define DSHOT_BIT_0     105     /* 37.5% duty = 625ns */
#define DSHOT_BIT_1     210     /* 75% duty = 1250ns */
#define DSHOT_BUF_SIZE  18      /* 16 data bits + 2 zero padding */

static uint16_t dshot_buf[DSHOT_BUF_SIZE];
static uint16_t motor_throttle[4] = {0, 0, 0, 0};  /* DSHOT 油门值 */

static void DSHOT_Encode(uint16_t throttle)
{
    uint16_t val = (throttle << 1) | 0;  /* telem = 0 */
    uint16_t crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0F;
    uint16_t frame = (val << 4) | crc;

    for (int i = 0; i < 16; i++)
        dshot_buf[i] = (frame >> (15 - i)) & 1 ? DSHOT_BIT_1 : DSHOT_BIT_0;
    dshot_buf[16] = 0;
    dshot_buf[17] = 0;
}

static void DSHOT_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 只配 PA8 -> TIM1_CH1，其他引脚不动（保持输入/高阻） */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    /* TIM1: PSC=0, ARR=279 -> 600kHz bit rate */
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Prescaler         = 0;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = DSHOT_ARR;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode       = TIM_OCMode_PWM1;
    oc.TIM_OutputState   = TIM_OutputState_Enable;
    oc.TIM_OutputNState  = TIM_OutputNState_Disable;
    oc.TIM_Pulse         = 0;
    oc.TIM_OCPolarity    = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity   = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState   = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState  = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* DMA2 Stream5 Channel6 = TIM1_UP -> CCR1 */
    DMA_DeInit(DMA2_Stream5);
    DMA_InitTypeDef dma;
    dma.DMA_Channel            = DMA_Channel_6;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&TIM1->CCR1;
    dma.DMA_Memory0BaseAddr    = (uint32_t)dshot_buf;
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = DSHOT_BUF_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &dma);

    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

static void DSHOT_SendFrame(uint16_t throttle)
{
    DSHOT_Encode(throttle);

    DMA_Cmd(DMA2_Stream5, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
    DMA_ClearFlag(DMA2_Stream5,
        DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 |
        DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
    DMA_SetCurrDataCounter(DMA2_Stream5, DSHOT_BUF_SIZE);
    DMA_Cmd(DMA2_Stream5, ENABLE);
}

/* MSP 油门值 (1000-2000) -> DSHOT 油门值 (0-2047) */
static uint16_t MSP_ToDshot(uint16_t msp_val)
{
    if (msp_val <= 1000) return 0;          /* disarm */
    if (msp_val >= 2000) return 2047;
    /* 1001-2000 -> 48-2047 */
    return 48 + (uint32_t)(msp_val - 1001) * (2047 - 48) / 999;
}

/* ========== Bit-Bang 串口 (19200 baud, 信号线) ========== */

/* 19200 baud 的 cycle 数 (168MHz) */
#define BIT_CYCLES          (SystemCoreClock / 19200)       /* 8750 */
#define BIT_CYCLES_HALF     (BIT_CYCLES / 2)                /* 4375 */
#define BIT_CYCLES_3_4      (BIT_CYCLES_HALF + BIT_CYCLES_HALF / 2)  /* 6562 */

/* Cycle-accurate delay: 等到 DWT->CYCCNT 到达 target */
static inline void DWT_WaitUntil(uint32_t target)
{
    while ((int32_t)(target - DWT->CYCCNT) > 0);
}

/* 从信号线读一个字节 (bit-bang UART RX) */
static uint8_t suart_getc(uint8_t *bt)
{
    uint32_t wait_end = DWT->CYCCNT + (uint32_t)START_BIT_TIMEOUT_MS * (SystemCoreClock / 1000);

    /* 等待 start bit (下降沿) */
    while (ESC_IsHi(selected_esc)) {
        if ((int32_t)(DWT->CYCCNT - wait_end) > 0) {
            return 0;  /* 超时 */
        }
    }

    /* start bit 检测到了，采样点在 3/4 bit time */
    uint32_t btime = DWT->CYCCNT + BIT_CYCLES_3_4;
    uint16_t bitmask = 0;
    uint8_t bit = 0;

    DWT_WaitUntil(btime);

    while (1) {
        if (ESC_IsHi(selected_esc)) {
            bitmask |= (1 << bit);
        }
        btime += BIT_CYCLES;
        bit++;
        if (bit == 10) break;
        DWT_WaitUntil(btime);
    }

    /* 校验 start bit (bit 0 应该是 0) 和 stop bit (bit 9 应该是 1) */
    if ((bitmask & 1) || (!(bitmask & (1 << 9)))) {
        return 0;
    }
    *bt = bitmask >> 1;
    return 1;
}

/* 向信号线写一个字节 (bit-bang UART TX) */
static void suart_putc(const uint8_t *tx_b)
{
    /* bitmask: stop(1) | data[7:0] | start(0) | idle(1) */
    uint16_t bitmask = (*tx_b << 2) | 1 | (1 << 10);
    uint32_t btime = DWT->CYCCNT;

    while (1) {
        if (bitmask & 1) {
            ESC_SetHi(selected_esc);
        } else {
            ESC_SetLo(selected_esc);
        }
        btime += BIT_CYCLES;
        bitmask >>= 1;
        if (bitmask == 0) break;
        DWT_WaitUntil(btime);
    }
}

/* ========== BLHeli Bootloader CRC ========== */

static uint8_16_u BL_CRC;
static uint8_16_u BL_LastCRC;

static void BL_ByteCrc(const uint8_t *bt)
{
    uint8_t xb = *bt;
    for (uint8_t i = 0; i < 8; i++) {
        if (((xb & 0x01) ^ (BL_CRC.word & 0x0001)) != 0) {
            BL_CRC.word = (BL_CRC.word >> 1) ^ 0xA001;
        } else {
            BL_CRC.word >>= 1;
        }
        xb >>= 1;
    }
}

/* ========== BLHeli Bootloader 通信 ========== */

static bool isMcuConnected(void)
{
    return (DeviceInfo.bytes[0] > 0);
}

static uint8_t BL_ReadBuf(uint8_t *pstring, uint8_t len)
{
    BL_CRC.word = 0;
    BL_LastCRC.word = 0;
    uint8_t LastACK = brNONE;

    do {
        if (!suart_getc(pstring)) goto timeout;
        BL_ByteCrc(pstring);
        pstring++;
        len--;
    } while (len > 0);

    if (isMcuConnected()) {
        if (!suart_getc(&BL_LastCRC.bytes[0])) goto timeout;
        if (!suart_getc(&BL_LastCRC.bytes[1])) goto timeout;
        if (!suart_getc(&LastACK)) goto timeout;
        if (BL_CRC.word != BL_LastCRC.word) {
            LastACK = brERRORCRC;
        }
    } else {
        if (!suart_getc(&LastACK)) goto timeout;
    }
timeout:
    return (LastACK == brSUCCESS);
}

static void BL_SendBuf(uint8_t *pstring, uint8_t len)
{
    ESC_SetOutput(selected_esc);
    BL_CRC.word = 0;

    do {
        suart_putc(pstring);
        BL_ByteCrc(pstring);
        pstring++;
        len--;
    } while (len > 0);

    if (isMcuConnected()) {
        suart_putc(&BL_CRC.bytes[0]);
        suart_putc(&BL_CRC.bytes[1]);
    }
    ESC_SetInput(selected_esc);
}

static uint8_t BL_GetACK(uint32_t timeoutMs)
{
    uint8_t LastACK = brNONE;
    uint32_t end = DWT->CYCCNT + timeoutMs * (SystemCoreClock / 1000);
    while ((int32_t)(end - DWT->CYCCNT) > 0) {
        if (suart_getc(&LastACK)) {
            return LastACK;
        }
    }
    return brNONE;
}

/* 连接 ESC Bootloader */
static uint8_t BL_ConnectEx(uint8_32_u *pDeviceInfo)
{
    uint8_t BootInfo[9];
    uint8_t BootMsg[3] = "471";

    uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    BL_SendBuf(BootInit, 17);

    if (!BL_ReadBuf(BootInfo, 4 + 4)) {
        return 0;
    }

    for (uint8_t i = 0; i < 3; i++) {
        if (BootInfo[i] != BootMsg[i]) {
            return 0;
        }
    }

    pDeviceInfo->bytes[2] = BootInfo[3];
    pDeviceInfo->bytes[1] = BootInfo[4];
    pDeviceInfo->bytes[0] = BootInfo[5];
    return 1;
}

static uint8_t BL_SendCMDKeepAlive(void)
{
    uint8_t sCMD[] = {BL_CMD_KEEP_ALIVE, 0};
    BL_SendBuf(sCMD, 2);
    if (BL_GetACK(4) != brERRORCOMMAND) {
        return 0;
    }
    return 1;
}

static void BL_SendCMDRunRestartBootloader(uint8_32_u *pDeviceInfo)
{
    uint8_t sCMD[] = {0, 0};  /* RestartBootloader */
    pDeviceInfo->bytes[0] = 1;
    BL_SendBuf(sCMD, 2);
}

static uint8_t BL_SendCMDSetAddress(ioMem_t *pMem)
{
    if ((pMem->D_FLASH_ADDR_H == 0xFF) && (pMem->D_FLASH_ADDR_L == 0xFF)) return 1;
    uint8_t sCMD[] = {BL_CMD_SET_ADDRESS, 0, pMem->D_FLASH_ADDR_H, pMem->D_FLASH_ADDR_L};
    BL_SendBuf(sCMD, 4);
    return (BL_GetACK(4) == brSUCCESS);
}

static uint8_t BL_SendCMDSetBuffer(ioMem_t *pMem)
{
    uint8_t sCMD[] = {BL_CMD_SET_BUFFER, 0, 0, pMem->D_NUM_BYTES};
    if (pMem->D_NUM_BYTES == 0) {
        sCMD[2] = 1;
    }
    BL_SendBuf(sCMD, 4);
    if (BL_GetACK(4) != brNONE) return 0;
    BL_SendBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES);
    return (BL_GetACK(80) == brSUCCESS);
}

static uint8_t BL_ReadA(uint8_t cmd, ioMem_t *pMem)
{
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {cmd, pMem->D_NUM_BYTES};
        BL_SendBuf(sCMD, 2);
        return BL_ReadBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES);
    }
    return 0;
}

static uint8_t BL_WriteA(uint8_t cmd, ioMem_t *pMem, uint32_t timeout)
{
    if (BL_SendCMDSetAddress(pMem)) {
        if (!BL_SendCMDSetBuffer(pMem)) return 0;
        uint8_t sCMD[] = {cmd, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK(timeout) == brSUCCESS);
    }
    return 0;
}

static uint8_t BL_ReadFlash(uint8_t interface_mode, ioMem_t *pMem)
{
    if (interface_mode == imATM_BLB) {
        return BL_ReadA(BL_CMD_READ_FLASH_ATM, pMem);
    } else {
        return BL_ReadA(BL_CMD_READ_FLASH_SIL, pMem);
    }
}

static uint8_t BL_ReadEEprom(ioMem_t *pMem)
{
    return BL_ReadA(BL_CMD_READ_EEPROM, pMem);
}

static uint8_t BL_PageErase(ioMem_t *pMem)
{
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {BL_CMD_ERASE_FLASH, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK(3000) == brSUCCESS);
    }
    return 0;
}

static uint8_t BL_WriteFlash(ioMem_t *pMem)
{
    return BL_WriteA(BL_CMD_PROG_FLASH, pMem, 500);
}

static uint8_t BL_WriteEEprom(ioMem_t *pMem)
{
    return BL_WriteA(BL_CMD_PROG_EEPROM, pMem, 3000);
}

static uint8_t BL_VerifyFlash(ioMem_t *pMem)
{
    if (BL_SendCMDSetAddress(pMem)) {
        if (!BL_SendCMDSetBuffer(pMem)) return 0;
        uint8_t sCMD[] = {BL_CMD_VERIFY_FLASH_ARM, 0x01};
        BL_SendBuf(sCMD, 2);
        return BL_GetACK(40);
    }
    return 0;
}

/* ========== 4-Way 连接管理 ========== */

#define SET_DISCONNECTED    DeviceInfo.words[0] = 0
#define INTF_MODE_IDX       3
#define SILABS_DEVICE_MATCH ((pDeviceInfo->words[0] > 0xE800) && (pDeviceInfo->words[0] < 0xF900))
#define ATMEL_DEVICE_MATCH  ((pDeviceInfo->words[0] == 0x9307) || (pDeviceInfo->words[0] == 0x930A) || \
                             (pDeviceInfo->words[0] == 0x930F) || (pDeviceInfo->words[0] == 0x940B))
#define ARM_DEVICE_MATCH    ((pDeviceInfo->bytes[1] > 0x00) && (pDeviceInfo->bytes[1] < 0x90) && (pDeviceInfo->bytes[0] == 0x06))

static uint8_t Connect(uint8_32_u *pDeviceInfo)
{
    for (uint8_t I = 0; I < 3; I++) {
        if (BL_ConnectEx(pDeviceInfo)) {
            if (SILABS_DEVICE_MATCH) {
                CurrentInterfaceMode = imSIL_BLB;
                return 1;
            } else if (ATMEL_DEVICE_MATCH) {
                CurrentInterfaceMode = imATM_BLB;
                return 1;
            } else if (ARM_DEVICE_MATCH) {
                CurrentInterfaceMode = imARM_BLB;
                return 1;
            }
        }
    }
    return 0;
}

/* ========== 4-Way CRC (XMODEM) ========== */

static uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

/* ========== MSP 协议处理 ========== */

static void MSP_SendResponse(uint8_t cmd, uint8_t *payload, uint8_t len)
{
    uint8_t crc = 0;

    UART_WriteByte('$');
    UART_WriteByte('M');
    UART_WriteByte('>');

    UART_WriteByte(len);
    crc ^= len;

    UART_WriteByte(cmd);
    crc ^= cmd;

    for (uint8_t i = 0; i < len; i++) {
        UART_WriteByte(payload[i]);
        crc ^= payload[i];
    }

    UART_WriteByte(crc);
    UART_Flush();
}

static void MSP_SendError(uint8_t cmd)
{
    uint8_t crc = 0;
    UART_WriteByte('$');
    UART_WriteByte('M');
    UART_WriteByte('!');
    UART_WriteByte(0);
    crc ^= 0;
    UART_WriteByte(cmd);
    crc ^= cmd;
    UART_WriteByte(crc);
    UART_Flush();
}

/* ========== 4-Way Interface 主循环 ========== */

static void FourWayProcess(void)
{
    uint8_t ParamBuf[258];
    uint8_t ESC_sel;
    uint8_t I_PARAM_LEN;
    uint8_t CMD;
    uint8_t ACK_OUT;
    uint8_16_u CRC_check;
    uint8_16_u CRC_in;
    uint8_16_u CRC_out;
    uint8_16_u Dummy;
    uint8_t O_PARAM_LEN;
    uint8_t *O_PARAM;
    ioMem_t ioMem;
    bool isExitScheduled = false;

    while (1) {
        /* 等待 cmd_Local_Escape (0x2F '/') */
        CRC_in.word = 0;
        do {
            ESC_sel = UART_ReadByte();
            CRC_in.word = crc_xmodem_update(CRC_in.word, ESC_sel);
        } while (ESC_sel != cmd_Local_Escape);

        Dummy.word = 0;
        O_PARAM = &Dummy.bytes[0];
        O_PARAM_LEN = 1;

        bool timedOut = false;

        /* 读 CMD */
        if (!UART_ReadByteTimeout(&CMD, 50000)) { timedOut = true; goto respond; }
        CRC_in.word = crc_xmodem_update(CRC_in.word, CMD);

        /* 读地址 H/L */
        if (!UART_ReadByteTimeout(&ioMem.D_FLASH_ADDR_H, 25000)) { timedOut = true; goto respond; }
        CRC_in.word = crc_xmodem_update(CRC_in.word, ioMem.D_FLASH_ADDR_H);

        if (!UART_ReadByteTimeout(&ioMem.D_FLASH_ADDR_L, 25000)) { timedOut = true; goto respond; }
        CRC_in.word = crc_xmodem_update(CRC_in.word, ioMem.D_FLASH_ADDR_L);

        /* 读参数长度 */
        if (!UART_ReadByteTimeout(&I_PARAM_LEN, 25000)) { timedOut = true; goto respond; }
        CRC_in.word = crc_xmodem_update(CRC_in.word, I_PARAM_LEN);

        /* 读参数数据 */
        {
            uint8_t i = I_PARAM_LEN;
            uint8_t *InBuff = ParamBuf;
            do {
                if (!UART_ReadByteTimeout(InBuff, 10000)) { timedOut = true; break; }
                CRC_in.word = crc_xmodem_update(CRC_in.word, *InBuff);
                InBuff++;
            } while (--i > 0);
        }

        /* 读 CRC */
        if (!timedOut) {
            if (!UART_ReadByteTimeout(&CRC_check.bytes[1], 10000)) timedOut = true;
        }
        if (!timedOut) {
            if (!UART_ReadByteTimeout(&CRC_check.bytes[0], 10000)) timedOut = true;
        }

        /* 校验 CRC */
        if ((CRC_check.word == CRC_in.word) && !timedOut) {
            ACK_OUT = ACK_OK;
        } else {
            ACK_OUT = ACK_I_INVALID_CRC;
        }

respond:
        if (timedOut) {
            ACK_OUT = ACK_I_INVALID_CRC;
        }

        if (ACK_OUT == ACK_OK) {
            ioMem.D_PTR_I = ParamBuf;

            switch (CMD) {
                case cmd_InterfaceTestAlive:
                    if (isMcuConnected()) {
                        if (!BL_SendCMDKeepAlive()) {
                            ACK_OUT = ACK_D_GENERAL_ERROR;
                        }
                        if (ACK_OUT != ACK_OK) SET_DISCONNECTED;
                    }
                    break;

                case cmd_ProtocolGetVersion:
                    Dummy.bytes[0] = SERIAL_4WAY_PROTOCOL_VER;
                    break;

                case cmd_InterfaceGetName:
                {
                    const char *name = "m4wFCIntf";
                    O_PARAM_LEN = strlen(name);
                    O_PARAM = (uint8_t *)name;
                    break;
                }

                case cmd_InterfaceGetVersion:
                    O_PARAM_LEN = 2;
                    Dummy.bytes[0] = SERIAL_4WAY_VERSION_HI;
                    Dummy.bytes[1] = SERIAL_4WAY_VERSION_LO;
                    break;

                case cmd_InterfaceExit:
                    isExitScheduled = true;
                    break;

                case cmd_InterfaceSetMode:
                    if ((ParamBuf[0] >= imSIL_BLB) && (ParamBuf[0] <= imARM_BLB)) {
                        CurrentInterfaceMode = ParamBuf[0];
                    } else {
                        ACK_OUT = ACK_I_INVALID_PARAM;
                    }
                    break;

                case cmd_DeviceReset:
                {
                    bool rebootEsc = false;
                    if (ParamBuf[0] < ESC_COUNT) {
                        selected_esc = ParamBuf[0];
                        if (ioMem.D_FLASH_ADDR_L == 1) {
                            rebootEsc = true;
                        }
                    } else {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    BL_SendCMDRunRestartBootloader(&DeviceInfo);
                    if (rebootEsc) {
                        ESC_SetOutput(selected_esc);
                        ESC_SetLo(selected_esc);
                        uint32_t m = DWT_GetMillis();
                        while ((DWT_GetMillis() - m) < 300);
                        ESC_SetHi(selected_esc);
                        ESC_SetInput(selected_esc);
                    }
                    SET_DISCONNECTED;
                    break;
                }

                case cmd_DeviceInitFlash:
                {
                    SET_DISCONNECTED;
                    if (ParamBuf[0] < ESC_COUNT) {
                        selected_esc = ParamBuf[0];
                    } else {
                        ACK_OUT = ACK_I_INVALID_CHANNEL;
                        break;
                    }
                    O_PARAM_LEN = 4;
                    O_PARAM = (uint8_t *)&DeviceInfo;
                    if (Connect(&DeviceInfo)) {
                        DeviceInfo.bytes[INTF_MODE_IDX] = CurrentInterfaceMode;
                    } else {
                        SET_DISCONNECTED;
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    break;
                }

                case cmd_DevicePageErase:
                {
                    Dummy.bytes[0] = ParamBuf[0];
                    if (CurrentInterfaceMode == imARM_BLB) {
                        ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 2);
                    } else {
                        ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 1);
                    }
                    ioMem.D_FLASH_ADDR_L = 0;
                    if (!BL_PageErase(&ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR;
                    break;
                }

                case cmd_DeviceRead:
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    if (!BL_ReadFlash(CurrentInterfaceMode, &ioMem)) {
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    if (ACK_OUT == ACK_OK) {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&ParamBuf;
                    }
                    break;

                case cmd_DeviceReadEEprom:
                    ioMem.D_NUM_BYTES = ParamBuf[0];
                    if (!BL_ReadEEprom(&ioMem)) {
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    if (ACK_OUT == ACK_OK) {
                        O_PARAM_LEN = ioMem.D_NUM_BYTES;
                        O_PARAM = (uint8_t *)&ParamBuf;
                    }
                    break;

                case cmd_DeviceWrite:
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    if (!BL_WriteFlash(&ioMem)) {
                        ACK_OUT = ACK_D_GENERAL_ERROR;
                    }
                    break;

                case cmd_DeviceWriteEEprom:
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    ACK_OUT = ACK_D_GENERAL_ERROR;
                    if (CurrentInterfaceMode == imATM_BLB) {
                        if (BL_WriteEEprom(&ioMem)) ACK_OUT = ACK_OK;
                    }
                    break;

                case cmd_DeviceVerify:
                    if (CurrentInterfaceMode == imARM_BLB) {
                        ioMem.D_NUM_BYTES = I_PARAM_LEN;
                        uint8_t verifyResult = BL_VerifyFlash(&ioMem);
                        if (verifyResult == brSUCCESS) ACK_OUT = ACK_OK;
                        else if (verifyResult == brERRORVERIFY) ACK_OUT = ACK_I_VERIFY_ERROR;
                        else ACK_OUT = ACK_D_GENERAL_ERROR;
                    } else {
                        ACK_OUT = ACK_I_INVALID_CMD;
                    }
                    break;

                case cmd_DeviceC2CK_LOW:
                    /* C2CK low - 仅 C2 模式有意义，这里仅拉低信号线 */
                    ESC_SetOutput(selected_esc);
                    ESC_SetLo(selected_esc);
                    break;

                default:
                    ACK_OUT = ACK_I_INVALID_CMD;
                    break;
            }
        }

        /* ---- 发送响应 ---- */
        CRC_out.word = 0;

        UART_WriteByte(cmd_Remote_Escape);
        CRC_out.word = crc_xmodem_update(CRC_out.word, cmd_Remote_Escape);

        UART_WriteByte(CMD);
        CRC_out.word = crc_xmodem_update(CRC_out.word, CMD);

        UART_WriteByte(ioMem.D_FLASH_ADDR_H);
        CRC_out.word = crc_xmodem_update(CRC_out.word, ioMem.D_FLASH_ADDR_H);

        UART_WriteByte(ioMem.D_FLASH_ADDR_L);
        CRC_out.word = crc_xmodem_update(CRC_out.word, ioMem.D_FLASH_ADDR_L);

        UART_WriteByte(O_PARAM_LEN);
        CRC_out.word = crc_xmodem_update(CRC_out.word, O_PARAM_LEN);

        {
            uint8_t i = O_PARAM_LEN;
            do {
                UART_WriteByte(*O_PARAM);
                CRC_out.word = crc_xmodem_update(CRC_out.word, *O_PARAM);
                O_PARAM++;
                i--;
            } while (i > 0);
        }

        UART_WriteByte(ACK_OUT);
        CRC_out.word = crc_xmodem_update(CRC_out.word, ACK_OUT);

        UART_WriteByte(CRC_out.bytes[1]);
        UART_WriteByte(CRC_out.bytes[0]);

        UART_Flush();

        if (isExitScheduled) {
            return;
        }
    }
}

/* ========== MSP 主循环 ========== */

/* 解析 MSP 帧并处理
 * 帧格式: '$' 'M' '<' len cmd [payload] crc */
static void MSP_ProcessLoop(void)
{
    enum { ST_IDLE, ST_M, ST_DIR, ST_LEN, ST_CMD, ST_DATA, ST_CRC } state = ST_IDLE;

    uint8_t msp_len = 0;
    uint8_t msp_cmd = 0;
    uint8_t msp_data[64];
    uint8_t msp_idx = 0;
    uint8_t msp_crc = 0;

    bool enter4way = false;
    uint32_t last_dshot = DWT->CYCCNT;
    uint32_t dshot_interval = SystemCoreClock / 1000;  /* 1ms = 1kHz 帧率 */

    while (!enter4way) {
        /* 非阻塞: 每 1ms 发一帧 DSHOT */
        uint32_t now = DWT->CYCCNT;
        if ((int32_t)(now - last_dshot) > (int32_t)dshot_interval) {
            DSHOT_SendFrame(motor_throttle[0]);
            last_dshot = now;
        }

        /* 非阻塞检查 UART 数据 */
        if (!UART_RxReady()) continue;
        uint8_t c = (uint8_t)(USART3->DR & 0xFF);

        switch (state) {
            case ST_IDLE:
                if (c == '$') state = ST_M;
                break;
            case ST_M:
                state = (c == 'M') ? ST_DIR : ST_IDLE;
                break;
            case ST_DIR:
                if (c == '<') state = ST_LEN;
                else state = ST_IDLE;
                break;
            case ST_LEN:
                msp_len = c;
                msp_crc = c;
                msp_idx = 0;
                state = ST_CMD;
                break;
            case ST_CMD:
                msp_cmd = c;
                msp_crc ^= c;
                if (msp_len > 0) state = ST_DATA;
                else state = ST_CRC;
                break;
            case ST_DATA:
                if (msp_idx < sizeof(msp_data)) {
                    msp_data[msp_idx] = c;
                }
                msp_crc ^= c;
                msp_idx++;
                if (msp_idx >= msp_len) state = ST_CRC;
                break;
            case ST_CRC:
            {
                state = ST_IDLE;
                if (msp_crc != c) {
                    /* CRC 错误，忽略 */
                    break;
                }

                /* 处理 MSP 命令 */
                switch (msp_cmd) {
                    case MSP_API_VERSION:
                    {
                        /* esc-configurator 首先查询 API 版本 */
                        uint8_t resp[] = {
                            0,      /* MSP protocol version */
                            1, 46   /* API version: 1.46 */
                        };
                        MSP_SendResponse(MSP_API_VERSION, resp, 3);
                        break;
                    }
                    case MSP_FC_VARIANT:
                    {
                        /* 返回 "BTFL" 让 esc-configurator 认为是 Betaflight */
                        uint8_t resp[] = {'B','T','F','L'};
                        MSP_SendResponse(MSP_FC_VARIANT, resp, 4);
                        break;
                    }
                    case MSP_FC_VERSION:
                    {
                        uint8_t resp[] = {4, 5, 0};  /* 4.5.0 */
                        MSP_SendResponse(MSP_FC_VERSION, resp, 3);
                        break;
                    }
                    case MSP_BOARD_INFO:
                    {
                        /* 简化 board info */
                        uint8_t resp[] = {'S','4','0','5', 0,0, 0, 0,0};
                        MSP_SendResponse(MSP_BOARD_INFO, resp, 9);
                        break;
                    }
                    case MSP_BUILD_INFO:
                    {
                        uint8_t resp[19];
                        memset(resp, 0, sizeof(resp));
                        MSP_SendResponse(MSP_BUILD_INFO, resp, 19);
                        break;
                    }
                    case MSP_UID:
                    {
                        /* 3 x uint32_t，用 MCU 的 96-bit unique ID */
                        uint8_t resp[12];
                        uint32_t *uid0 = (uint32_t *)0x1FFF7A10;
                        uint32_t *uid1 = (uint32_t *)0x1FFF7A14;
                        uint32_t *uid2 = (uint32_t *)0x1FFF7A18;
                        memcpy(&resp[0], uid0, 4);
                        memcpy(&resp[4], uid1, 4);
                        memcpy(&resp[8], uid2, 4);
                        MSP_SendResponse(MSP_UID, resp, 12);
                        break;
                    }
                    case MSP_MOTOR:
                    {
                        /* 8 个电机的油门值 (uint16_t each)，全部返回 0 */
                        uint8_t resp[16];
                        memset(resp, 0, sizeof(resp));
                        /* 前4个电机返回非零值，让 esc-configurator 知道有4路 */
                        resp[0] = 0xE8; resp[1] = 0x03; /* 1000 little-endian */
                        resp[2] = 0xE8; resp[3] = 0x03;
                        resp[4] = 0xE8; resp[5] = 0x03;
                        resp[6] = 0xE8; resp[7] = 0x03;
                        MSP_SendResponse(MSP_MOTOR, resp, 16);
                        break;
                    }
                    case MSP_FEATURE_CONFIG:
                    {
                        /* uint32_t feature bits，全部返回 0 */
                        uint8_t resp[4] = {0, 0, 0, 0};
                        MSP_SendResponse(MSP_FEATURE_CONFIG, resp, 4);
                        break;
                    }
                    case MSP_SET_MOTOR:
                    {
                        /* 16 bytes: 8 motors × uint16_t little-endian */
                        if (msp_len >= 8) {
                            uint16_t m1 = msp_data[0] | (msp_data[1] << 8);
                            uint16_t m2 = msp_data[2] | (msp_data[3] << 8);
                            uint16_t m3 = msp_data[4] | (msp_data[5] << 8);
                            uint16_t m4 = msp_data[6] | (msp_data[7] << 8);
                            motor_throttle[0] = MSP_ToDshot(m1);
                            motor_throttle[1] = MSP_ToDshot(m2);
                            motor_throttle[2] = MSP_ToDshot(m3);
                            motor_throttle[3] = MSP_ToDshot(m4);
                        }
                        /* 返回空响应 */
                        MSP_SendResponse(MSP_SET_MOTOR, NULL, 0);
                        break;
                    }
                    case MSP_SET_PASSTHROUGH:
                    {
                        /* 进入 4-way interface 模式 */
                        /* 初始化 ESC 引脚 */
                        ESC_PinsInit();
                        uint8_t resp[] = {ESC_COUNT};
                        MSP_SendResponse(MSP_SET_PASSTHROUGH, resp, 1);
                        UART_Flush();
                        enter4way = true;
                        break;
                    }
                    default:
                        /* 未知命令，返回错误 */
                        MSP_SendError(msp_cmd);
                        break;
                }
                break;
            }
        }
    }
}

/* ========== 公开接口 ========== */

void Passthrough_Init(void)
{
    DWT_Init();
    USART3_Init_115200();

    /* 延迟 DSHOT 初始化，让 ESC 先完成 boot（启动音） */
    /* 此时信号引脚为默认状态（输入/高阻），不干扰 ESC 启动 */
    {
        uint32_t boot_start = DWT->CYCCNT;
        uint32_t boot_wait = (uint32_t)4 * SystemCoreClock;  /* 等 4 秒 */
        while ((int32_t)(DWT->CYCCNT - boot_start) < (int32_t)boot_wait);
    }

    DSHOT_Init();  /* ESC 启动完成后再配置 DSHOT 输出 */

    /* LED 指示 (PB9) */
    {
        GPIO_InitTypeDef gpioInit;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        gpioInit.GPIO_Pin   = GPIO_Pin_9;
        gpioInit.GPIO_Mode  = GPIO_Mode_OUT;
        gpioInit.GPIO_Speed = GPIO_Speed_2MHz;
        gpioInit.GPIO_OType = GPIO_OType_PP;
        gpioInit.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &gpioInit);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);  /* LED 亮 = 等待连接 */
    }
}

void Passthrough_Run(void)
{
    /* 阶段 1: MSP 协议解析，等待 esc-configurator 发送 MSP_SET_PASSTHROUGH */
    MSP_ProcessLoop();

    /* 阶段 2: 进入 4-way interface 模式 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);  /* LED 灭 = 已进入 4-way 模式 */
    FourWayProcess();

    /* 退出 4-way 模式 (理论上不会到这里，除非 esc-configurator 主动退出) */
    GPIO_SetBits(GPIOB, GPIO_Pin_9);  /* LED 亮 = 回到空闲 */
}