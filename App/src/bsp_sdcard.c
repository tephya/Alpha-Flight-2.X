#include "bsp_sdcard.h"
#include "main.h"
#include <string.h>


#define SD_CS_LOW() HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)

#define SD_DMA_TIMEOUT_MS 20U
#define SD_BUSY_TIMEOUT_MS 500U

extern SPI_HandleTypeDef hspi2;

static SdCardType_t s_sdCardType = SD_TYPE_UNKNOWN;
static osSemaphoreId_t s_sdXferSem = NULL;
static volatile uint8_t s_sdDmaError = 0;

// DMA全双工传输时，不关心的一侧用这两个静态scratch缓冲填充，避免每次现场申请栈内存
static uint8_t s_sdTxDummy[512];
static uint8_t s_sdRxSink[512];

static void SD_CS_High(void)
{
    uint8_t rx;
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    HAL_SPI_TransmitReceive(&hspi2, s_sdTxDummy, &rx, 1, 10);       // 额外1个clock(经验做法，目的是等信号稳定)
}

/**
 * @brief   底层阻塞式SPI单字节全双工收发，仅用于命令/相应这类短小、不值得上DMA的场景
 */
static uint8_t SD_SPI_RWByte(uint8_t txd)
{
    uint8_t rxd = 0xFF;
    HAL_SPI_TransmitReceive(&hspi2, &txd, &rxd, 1, 10);
    return rxd;
}

/**
 * @brief   发送标准6字节SD命令并等待R1响应
 */
static uint8_t SD_SendCmd(uint8_t cmd ,uint32_t arg)
{
    uint8_t crc = 0x01;
    if(cmd == 0)
        crc = 0x95;
    if(cmd == 8)
        crc = 0x87;

    SD_SPI_RWByte(0x40 | cmd);
    SD_SPI_RWByte((arg >> 24) & 0xFF);
    SD_SPI_RWByte((arg >> 16) & 0xFF);
    SD_SPI_RWByte((arg >> 8) & 0xFF);
    SD_SPI_RWByte(arg & 0xFF);
    SD_SPI_RWByte(crc);

    uint8_t res = 0xFF;
    for (int i = 0; i < 8; i++)
    {
        res = SD_SPI_RWByte(0xFF);
        if((res & 0x80) == 0)
            break;
    }
    return res;
}

/**
 * @brief   DMA全双工收发len字节，tx/rx任一方向不关心时传scratch缓冲
 * @note    调用方保证len<=512(scratch缓冲大小上限，SD卡Block本身也是512)
 */
static int8_t SD_SPI_DMA_Transceive(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if(len == 0 || len > sizeof(s_sdTxDummy))
        return -1;

    s_sdDmaError = 0;

    if(HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)tx, rx, len) != HAL_OK)
        return -2;

    osStatus_t st = osSemaphoreAcquire(s_sdXferSem, SD_DMA_TIMEOUT_MS);
    if(st != osOK)
        return -3;      // 超时

    if(s_sdDmaError)
        return -4;

    return 0;
}

/**
 * @brief   HAL_SPI_DMA全双工完成回调(全工程仅这一处实现，按Instance分流)
 * @note    项目当前只有SD卡这一路SPI走DMA，若以后有第二路DMA_SPI，
 *          就在CubeMX中统一定义回调
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        osSemaphoreRelease(s_sdXferSem);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        s_sdDmaError = 1;
        osSemaphoreRelease(s_sdXferSem);
    }
}

/**
 * @brief   初始化SD卡SPI层软件状态（信号量）+ 执行SD卡上电时序(CMD0/CMD8/ACMD41/CMD58)
 * @note    SPI2外设本身由CubeMX的MX_SPI2_Init()在调度器启动前完成初始化，
 *          本函数只负责创建DMA完成信号量、执行卡上电协议、判定SDSC/SDHC类型。
 *          必须在osKernelStart()之后、由某个Task调用(内部osDelay需要调度器运行)。
 *          初始化失败除了括号说明外，还可能与SD卡内部状态机未复位/未进入工作状态，卡损坏等有关。
 * @retval  0 : 成功
 *         -1 : CMD0无响应(卡未插入/接线问题)
 *         -2 : CMD8无响应(不支持SD 2.0协议的卡，本工程不支持)
 *         -3 : CMD8校验模式回显不匹配
 *         -4 : ACMD41超时(卡未完成内部初始化)
 */
int8_t BSP_SD_Init(void)
{
    uint8_t res, buf[4];

    s_sdXferSem = osSemaphoreNew(1, 0, NULL);
    memset(s_sdTxDummy, 0xFF, sizeof(s_sdTxDummy));

    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    for (int i = 0; i < 10; i++)
        SD_SPI_RWByte(0xFF);        // >=74 dummy clock

    // CMD0 带重试
    res = 0xFF;
    for (int i = 0; i < 10; i++)
    {
        SD_CS_LOW();
        res = SD_SendCmd(0, 0x00000000);
        SD_CS_High();
        if(res == 0x01)
            break;
    }
    if(res != 0x01)
        return -1;

    // CMD8
    SD_CS_LOW();
    res = SD_SendCmd(8, 0x000001AA);
    for (int i = 0; i < 4; i++)
        buf[i] = SD_SPI_RWByte(0xFF);
    SD_CS_High();
    if(res != 0x01)
        return -2;
    if(buf[3] != 0xAA)
        return -3;

    // ACMD41
    res = 0xFF;
    for (int i = 0; i < 500; i++)
    {
        SD_CS_LOW();
        SD_SendCmd(55, 0x00000000);
        SD_CS_High();

        SD_CS_LOW();
        res = SD_SendCmd(41, 0x40000000);
        SD_CS_High();
        if(res == 0x00)
            break;

        osDelay(1);
    }
    if(res != 0x00)
        return -4;

    // CMD58 判定SDHC/SDSC
    SD_CS_LOW();
    SD_SendCmd(58, 0x00000000);
    for (int i = 0; i < 4; i++)
        buf[i] = SD_SPI_RWByte(0xFF);
    SD_CS_High();

    s_sdCardType = (buf[0] & 0x40) ? SD_TYPE_SDHC : SD_TYPE_SDSC;

    return 0;
}

/**
 * @brief   切换SPI2波特率分频(初始化用低速，握手完成后切全速)
 * @param   prescaler   SPI_BAUDRATEPRESCALER_x(HAL宏)
 */
void BSP_SD_SetSpeed(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi2);
    hspi2.Instance->CR1 = (hspi2.Instance->CR1 & ~SPI_CR1_BR) | prescaler;
    __HAL_SPI_ENABLE(&hspi2);
}

/**
 * @brief   读取单个512字节Block(全程走DMA)
 * @note    寻址逻辑：SDHC卡block参数直接当Block号；SDSC卡内部转换为字节地址(Blcok*512)。
 *          时序：CMD17取R1 ->
 *               轮询等待Data Token(0xFE) ->
 *               DMA收512字节 ->
 *               丢弃2字节CRC
 * @param   block   Block索引(SDHC=Block号，SDSC内部自动转为字节地址)
 * @param   buf     接收缓冲区，至少512字节
 * @retval  0 : 成功
 *         -1 : CMD17发送200次仍未收到R1=0x00响应(命令被拒绝/卡未就绪)
 *         -2 : R1正常轮询500次仍未等到Data Token(0xFE)，读取超时
 *         -3 : DMA数据段传输失败(SPI_DMA_Transceive内部超时或DMA错误)
 */
int8_t BSP_SD_ReadBlock(uint32_t block, uint8_t *buf)
{
    uint8_t res = 0xFF;
    uint32_t addr = (s_sdCardType == SD_TYPE_SDHC) ? block : block * 512U;

    SD_CS_LOW();

    for (int i = 0; i < 200; i++)
    {
        res = SD_SendCmd(17, addr);
        if(res == 0x00)
            break;
    }
    if(res != 0x00){ SD_CS_High(); return -1; }

    for (int i = 0; i < 500; i++)
    {
        res = SD_SPI_RWByte(0xFF);
        if(res == 0xFE)
            break;
    }
    if(res != 0xFE){ SD_CS_High(); return -2; }

    if(SD_SPI_DMA_Transceive(s_sdTxDummy, buf, 512) != 0)
    {
        SD_CS_High();
        return -3;
    }

    SD_SPI_RWByte(0xFF);    // CRC1
    SD_SPI_RWByte(0xFF);    // CRC2

    SD_CS_High();
    return 0;
}

/**
 * @brief   写入单个512字节Block(全程走DMA)
 * @note    时序：CMD24取R1 -> 
 *               发Data Token(0xFE) ->
 *               DMA发512字节 ->
 *               发2字节dummy CRC ->
 *               轮询Data Response确认卡是否接收(0x05) ->
 *               轮询MISO直到卡内部擦写结束(脱离Busy)。
 * @param   block   Block索引(SDHC=Block号，SDSC内部自动转为字节地址)
 * @param   buf     待写入数据，512字节
 * @retval  0 : 成功
 *         -1 : CMD24发送200次仍未收到R1=0x00响应
 *         -2 : DMA数据段传输失败(含Data Token发送后的512字节实际数据)
 *         -3 : Data Response校验异常(卡拒接写入的数据，如CRC错误/写保护)
 *         -4 : 等待卡内部擦写结束超时(SD_BUSY_TIMEOUT_MS=500ms内MISO未回高电平，
 *              肯是卡本身写入慢或损坏)
 */
int8_t BSP_SD_WriteBlock(uint32_t block, const uint8_t *buf)
{
    uint8_t res = 0xFF;
    uint32_t addr = (s_sdCardType == SD_TYPE_SDHC) ? block : block * 512U;

    SD_CS_LOW();

    for (int i = 0; i < 200; i++)
    {
        res = SD_SendCmd(24, addr);
        if(res == 0x00)
            break;
    }
    if(res != 0x00){ SD_CS_High(); return -1; }

    SD_SPI_RWByte(0xFF);
    SD_SPI_RWByte(0xFE);        // Data Token

    if(SD_SPI_DMA_Transceive(buf, s_sdRxSink, 512) != 0)
    {
        SD_CS_High();
        return -2;
    }

    SD_SPI_RWByte(0xFF);        // CRC1
    SD_SPI_RWByte(0xFF);        // CRC2

    res = 0xFF;
    for (int i = 0; i < 10; i++)
    {
        res = SD_SPI_RWByte(0xFF);
        if((res & 0x11) == 0x01)
            break;
    }
    if((res & 0x1F) != 0x05){ SD_CS_High(); return -3; }

    uint32_t start = osKernelGetTickCount();
    do
    {
        res = SD_SPI_RWByte(0xFF);
        if((osKernelGetTickCount() - start) >= SD_BUSY_TIMEOUT_MS)
        {
            SD_CS_High();
            return -4;
        }
    } while (res == 0x00);

    SD_CS_High();
    return 0;
}
