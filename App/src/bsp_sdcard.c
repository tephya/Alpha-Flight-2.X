/**
 * @file    bsp_sdcard.c
 * @brief   SD Card SPI Mode 初始化及单 Block DMA 读写实现。
 */

#include "bsp_sdcard.h"
#include "main.h"
#include <string.h>
/** 拉低 SD CS，选中 Card。 */
#define SD_CS_LOW()      \
    HAL_GPIO_WritePin(   \
        SD_CS_GPIO_Port, \
        SD_CS_Pin,       \
        GPIO_PIN_RESET)

#define SD_DMA_TIMEOUT_MS 20U       // 单次 SPI DMA Transfer 最大等待时间，ms。
#define SD_BUSY_TIMEOUT_MS 500U     // SD Card 内部 Programming Busy 最大等待时间，ms。
#define SD_BLOCK_SIZE 512U          // SD Block 固定长度，Byte。
#define SD_SPI_MAX_HZ 2625000UL     // 运行阶段允许的最高 SPI Clock，Hz。

extern SPI_HandleTypeDef hspi2;

static SdCardType_t s_sdCardType = SD_TYPE_UNKNOWN;     // 当前识别到的 SD Card 地址类型。
static osSemaphoreId_t s_sdXferSem = NULL;      // SPI2 DMA 完成同步信号量。
static volatile uint8_t s_sdDmaError = 0;       // 本轮 SPI DMA 是否发生错误。

/*
 * SPI DMA 为 Full-Duplex。
 * 
 * 当业务只关心 RX 时，TX 端持续发送 0xFF；
 * 当业务只关心 TX 时，RX 端数据写入 Sink Buffer。
 * 
 * 两个 Buffer 均放入 DMA 可访问 SRAM，避免在调用现场申请大块 Stack。
 */
#pragma arm section zidata = "DMA_SAFE_SRAM"

static uint8_t s_sdTxDummy[512];
static uint8_t s_sdRxSink[512];

#pragma arm section zidata

/*
 * 释放 SD CS。
 * 
 * CS 拉高后额外发送一个 0xFF Byte，
 * 为 Card 提供额外 SPI Clock，使一次 Transaction 完整结束并释放 Bus。
 */
static void SD_CS_High(void)
{
    uint8_t rx;

    HAL_GPIO_WritePin(
        SD_CS_GPIO_Port,
        SD_CS_Pin,
        GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(
        &hspi2,
        s_sdTxDummy,
        &rx,
        1U,
        10U);
}

/*
 * 阻塞式收发单个 SPI Byte。
 * 
 * 仅用于 Command，R1，Token，CRC 等短数据阶段；
 * 512 Byte Payload 才使用 DMA。
 */
static uint8_t SD_SPI_RWByte(uint8_t txd)
{
    uint8_t rxd = 0xFF;

    HAL_SPI_TransmitReceive(
        &hspi2,
        &txd,
        &rxd,
        1U,
        10U);

    return rxd;
}

/*
 * 发送一条标准 6 Byte SD Command，并等待 R1 Response。
 * 
 * SPI Mode Command Format：
 * 
 * [0x40 | CMD]
 * [ARG31:24]
 * [ARG23:16]
 * [ARG15:8]
 * [ARG7:0]
 * 
 * SPI Mode 初始化完成后通常不要求有效 CRC，
 * 但 CMD0 / CMD8 在初始化阶段使用协议规定的固定 CRC。
 */
static uint8_t SD_SendCmd(uint8_t cmd ,uint32_t arg)
{
    uint8_t crc = 0x01U;

    if (cmd == 0U)
    {
        crc = 0x95U;
    }

    if (cmd == 8U)
    {
        crc = 0x87U;
    }

    SD_SPI_RWByte(0x40 | cmd);
    SD_SPI_RWByte((arg >> 24) & 0xFF);
    SD_SPI_RWByte((arg >> 16) & 0xFF);
    SD_SPI_RWByte((arg >> 8) & 0xFF);
    SD_SPI_RWByte(arg & 0xFF);
    SD_SPI_RWByte(crc);

    /*
     * R1 bit7 为 0 时表示 Response Byte 已经达到。
     * Card 在 Command 后可能延迟若干 Byte 才输出 R1。
     */
    uint8_t res = 0xFF;

    for (int i = 0; i < 8; i++)
    {
        res = SD_SPI_RWByte(0xFF);
        if((res & 0x80) == 0)
            break;
    }
    return res;
}

/*
 * 使用 SPI2 DMA 全双工传输 len Byte。
 * 
 * SPI 硬件时钟同时 TX / RX：
 * 
 * - 只读时：tx 指向 s_sdTxDummy:
 * - 只写时：rx 指向 s_sdRxSink。
 * 
 * 调用方保证 len 不超过 Scratch Buffer 容量。
 */
static int8_t SD_SPI_DMA_Transceive(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if(len == 0 || len > sizeof(s_sdTxDummy))
        return -1;

    s_sdDmaError = 0;

    /*
     * 发起新 DMA 前先尝试清掉可能遗留的 Semaphore Token。
     * 
     * 正常情况下 Acquire(0) 应立即失败；
     * 如果成功，说明存在上一轮未消费的完成信号，
     * 将其丢弃，变无盘为本轮 DMA 已完成。
     */
    osSemaphoreAcquire(s_sdXferSem, 0);

    if (HAL_SPI_TransmitReceive_DMA(
            &hspi2,
            (uint8_t *)tx,
            rx,
            len) != HAL_OK)
    {
        return -2;
    }

    /*
     * Task 阻塞等待 SPI DMA Complete / Error Callback 释放 Semaphore。
     * 等待期间 CPU 可调度其他 Task，不进行 Busy Wait。
     */
    const osStatus_t status = osSemaphoreAcquire(s_sdXferSem, SD_DMA_TIMEOUT_MS);
    if(status != osOK)
    {
        return -3; 
    }

    if(s_sdDmaError != 0U)
        return -4;

    return 0;
}

/*
 * SPI Full-Duplex DMA Complete HAL Callback。
 * 
 * 当前 SPI2 DMA 属于 SD Card 数据链，
 * 完成后释放 Semaphore 唤醒正在等待的 SD Task。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        osSemaphoreRelease(s_sdXferSem);
    }
}

/*
 * SPI HAL Error Callback。
 * 
 * 先记录 DMA Error，再释放 Semaphore，
 * 使等待中的 Task 能及时退出并返回错误。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        s_sdDmaError = 1;
        osSemaphoreRelease(s_sdXferSem);
    }
}

int8_t BSP_SD_Init(void)
{
    uint8_t res;
    uint8_t buf[4];

    s_sdCardType = SD_TYPE_UNKNOWN;

    s_sdXferSem = osSemaphoreNew(1, 0, NULL);

    /*
     * SPI Read 时需要主动发送 Clock，
     * 因此 Dummy TX Buffer 固定填充为 0xFF。
     */
    memset(s_sdTxDummy, 0xFF, sizeof(s_sdTxDummy));

    /*
     * MCU Reset 不一定会让 SD Card 同时掉电。
     * 
     * Card 可能仍停留在上一次 Transaction 的中间状态，
     * 因此先等待一小段时间，并在 CS High 状态下提供至少 74 个 Clock (@ 100~400 kHz)，
     * 使 Card 有机会恢复并进入 SPI Mode 初始化入口。
     */
    osDelay(10U);

    HAL_GPIO_WritePin(
        SD_CS_GPIO_Port,
        SD_CS_Pin,
        GPIO_PIN_SET);

    for (uint8_t i = 0U;
         i < 10U;
         i++)
    {
        SD_SPI_RWByte(0xFFU);
    }

    /*
     * CMD0：请求进入 Idle State。
     * 
     * 使用有限次数重试，兼容 Card 在 MCU Reset 后仍处于旧状态，
     * 尚未完全恢复到可接受新 Command 的情况。
     */
    res = 0xFFU;

    for (int i = 0U; i < 10U; i++)
    {
        SD_CS_LOW();

        res = SD_SendCmd(0U, 0x00000000UL);

        SD_CS_High();
		
		SD_SPI_RWByte(0xFFU);
        
		if(res == 0x01U)
            break;

        osDelay(2U);
    }

    if (res != 0x01U)
    {
        return -1;
    }

    /*
     * CMD8：确认 Card 支持当前初始化流程，
     * 并检查 Check Pattern 0xAA 是否被正确回显。
     */
    SD_CS_LOW();

    res = SD_SendCmd(8U, 0x000001AAUL);

    for (int i = 0U; i < 4U; i++)
        buf[i] = SD_SPI_RWByte(0xFFU);

    SD_CS_High();

    if(res != 0x01U)
        return -2;

    if(buf[3] != 0xAAU)
        return -3;

    /*
     * ACMD41：
     * 
     * CMD55 表示下一条 Command 为 Application Specific Command：
     * 随后的 ACMD41 请求 Card 完成内部初始化。
     * 
     * HCS bit (ACMD41 arg bit30) =1，请求支持 High Capacity Card(HCS)。
     */
    res = 0xFF;
    
    for (int i = 0; i < 500; i++)
    {
        SD_CS_LOW();

        (void)SD_SendCmd(
            55U,
            0x00000000UL);

        SD_CS_High();

        SD_CS_LOW();

        res =
            SD_SendCmd(
                41U,
                0x40000000UL);

        SD_CS_High();
        
        if (res == 0x00U)
        {
            break;
        }

        osDelay(1);
    }

    if (res != 0x00U)
    {
        return -4;
    }

    /*
     * CMD58：读取 OCR(Operating Conditions Register)。
     *
     * OCR CCS bit：
     * 1 -> SDHC / SDXC，Block Addressing；
     * 0 -> SDSC，Byte Addressing。
     */
    SD_CS_LOW();

    (void)SD_SendCmd(
        58U,
        0x00000000UL);
        
    for (int i = 0; i < 4; i++)
        buf[i] = SD_SPI_RWByte(0xFFU);

    SD_CS_High();

    s_sdCardType =
        (buf[0] & 0x40U)
            ? SD_TYPE_SDHC
            : SD_TYPE_SDSC;

    return 0;
}

int8_t BSP_SD_ReadBlock(uint32_t block, uint8_t *buf)
{
    if (buf == NULL)
    {
        return -4;
    }

    uint8_t res = 0xFF;

    /*
     * SDHC 使用 Block Address；
     * SDSC 使用 Byte Address。
     */
    const uint32_t addr =
        (s_sdCardType == SD_TYPE_SDHC)
            ? block
            : block * SD_BLOCK_SIZE;

    /*
     * CMD17：Single Block Read。
     * 
     * Card 接受命令后返回 R1=0x00，
     * 随后通过 0xFE Data Token 表示 512 Byte Payload 即将开始。
     */
    SD_CS_LOW();

    for (int i = 0U; i < 200U; i++)
    {
        res =
            SD_SendCmd(
                17U,
                addr);

        if (res == 0x00U)
        {
            break;
        }

        osDelay(2U);
    }

    if (res != 0x00U)
    {
        SD_CS_High();
        return -1;
    }

    /* 等待 Single Block Data Token 0xFE。 */
    for (int i = 0U; i < 500U; i++)
    {
        res =
            SD_SPI_RWByte(0xFFU);

        if (res == 0xFEU)
        {
            break;
        }
    }

    if (res != 0xFEU)
    {
        SD_CS_High();
        return -2;
    }

    /*
     * SPI Read 仍需要 TX Clock，
     * 因此 DMA TX 使用全 0xFF Dummy Buffer。
     */
    if (SD_SPI_DMA_Transceive(
            s_sdTxDummy,
            buf,
            SD_BLOCK_SIZE) != 0)
    {
        SD_CS_High();
        return -3;
    }

    /* SPI Mode Data Block 尾部包含 2 Byte CRC。 */
    (void)SD_SPI_RWByte(0xFFU);
    (void)SD_SPI_RWByte(0xFFU);

    SD_CS_High();

    return 0;
}

int8_t BSP_SD_WriteBlock(uint32_t block, const uint8_t *buf)
{
    if (buf == NULL)
    {
        return -5;
    }

    uint8_t res = 0xFFU;

    const uint32_t addr =
        (s_sdCardType == SD_TYPE_SDHC)
            ? block
            : block * SD_BLOCK_SIZE;

    /*
     * CMD24：Single Block Write。
     * R1=0x00 表示 Card 接受本次 Write Command。
     */
    SD_CS_LOW();

    for (int i = 0U; i < 200U; i++)
    {
        res =
            SD_SendCmd(
                24U,
                addr);

        if (res == 0x00U)
        {
            break;
        }

        osDelay(2U);
    }

    if (res != 0x00U)
    {
        SD_CS_High();
        return -1;
    }

    /*
     * 在 Data Token 前提供一个 Dummy Byte，
     * 随后发送 Single Block Write Token 0xFE。
     */
    SD_SPI_RWByte(0xFF);
    SD_SPI_RWByte(0xFE);

    if (SD_SPI_DMA_Transceive(
            buf,
            s_sdRxSink,
            SD_BLOCK_SIZE) != 0)
    {
        SD_CS_High();
        return -2;
    }

    /*
     * 当前 SPI Mode 下不计算 Payload CRC，
     * 发送两个 Dummy CRC Byte。
     */
    SD_SPI_RWByte(0xFFU);
    SD_SPI_RWByte(0xFFU);

    /*
     * 等待 Data Response Token。
     * 
     * 低 5 bit = 0x05 表示 Data Accepted。
     */
    res = 0xFFU;

    for (int i = 0U; i < 10U; i++)
    {
        res =
            SD_SPI_RWByte(0xFFU);

        if ((res & 0x11U) == 0x01U)
        {
            break;
        }
    }

    if ((res & 0x1FU) != 0x05U)
    {
        SD_CS_High();
        return -3;
    }

    /*
     * Card 接收完 512 Byte 后还需要内部 Programming。
     * 
     * Busy 期间 MISO 保持 Low：
     * 当再次读到非 0x00 时表示 Card 已释放 Busy。
     */
    const uint32_t start = osKernelGetTickCount();

    do
    {
        res =
            SD_SPI_RWByte(0xFFU);

        if ((uint32_t)(osKernelGetTickCount() -
                       start) >=
            SD_BUSY_TIMEOUT_MS)
        {
            SD_CS_High();
            return -4;
        }
    } while (res == 0x00U);

    SD_CS_High();
    return 0;
}

/*
 * 修改 SPI2 Baud Rate Prescaler。
 * 
 * 仅改变 BR 位，不重新执行完整 HAL SPI Init。
 * 修改前关闭 SPI，写入完成后重新使能。
 */
static void BSP_SD_SetSpeed(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi2);

    hspi2.Instance->CR1 =
        (hspi2.Instance->CR1 &
         ~SPI_CR1_BR) |
        prescaler;

    __HAL_SPI_ENABLE(&hspi2);
}

void BSP_SD_SetSpeedFast(void)
{
    /*
     * SPI2 Clock 直接来源于 APB1 Peripheral Clock，
     * 不存在 Timer Peripheral 的 “APB Prescaler ！= 1 时 ×2”规则。
     */
    static const uint32_t s_divs[] =
        {
            2U,
            4U,
            8U,
            16U,
            32U,
            64U,
            128U,
            256U};

    static const uint32_t s_prescs[] =
        {
            SPI_BAUDRATEPRESCALER_2,
            SPI_BAUDRATEPRESCALER_4,
            SPI_BAUDRATEPRESCALER_8,
            SPI_BAUDRATEPRESCALER_16,
            SPI_BAUDRATEPRESCALER_32,
            SPI_BAUDRATEPRESCALER_64,
            SPI_BAUDRATEPRESCALER_128,
            SPI_BAUDRATEPRESCALER_256};

    uint32_t pclk = HAL_RCC_GetPCLK1Freq();

    /* 找不到合适档位时的保守兜底。 */
    uint32_t chosen = SPI_BAUDRATEPRESCALER_256;

    for (uint8_t i = 0U; i < sizeof(s_divs) / sizeof(s_divs[0]); i++)
    {
        if ((pclk / s_divs[i]) <=
            SD_SPI_MAX_HZ)
        {
            chosen =
                s_prescs[i];

            break;
        }
    }

    BSP_SD_SetSpeed(chosen);
}
