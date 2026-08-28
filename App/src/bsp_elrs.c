/**
 * @file    bsp_elrs.c
 * @brief   基于 UART DMA Circular Buffer 的 CRSF 接收与解析实现。
 */

#include "bsp_elrs.h"
#include "usart.h"
#include "cmsis_os2.h"

#define BUFF_SIZE 64        // CRSF 完整 Frame 最大长度，Byte。
#define DMA_BUF_SIZE 128    // UART DAM Circular RX Buffer 长度，Byte。

/*
 * RC Link Failsafe Timeout，ms。
 * 
 * 只有成功通过 CRC 校验的 RC Channel Frame 才会刷新该计时器；
 * Link Statistics 等其他 CRSF Frame 不参与 RC link 存活判断。
 */
#define CRSF_FAILSAFE_TIMEOUT_MS 200U

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U    // CRSF Flight Controller Device Address。
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16U // CRSF RC Channels Packed Frame Type。
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14U    // CRSF Link Statistics Frame Type。

/**
 * @brief   CRSF 字节流组帧状态。
 */
typedef enum
{
    WAIT_SYNC, /**< 等待 Frame Address / Sync。 */
    READ_LEN,  /**< 等待 Length 字段。 */
    READ_DATA  /**< 接收 Type + Payload + CRC。 */
} RCState_t;

static RCState_t s_state = WAIT_SYNC;   // 当前 CRSF Parser 状态。
static uint8_t s_idx = 0;               // 当前 Frame Buffer 写入位置。
static uint8_t s_frame_len = 0;         // 当前完整 CRSF Frame 长度，Byte。
static uint8_t s_frame_ready = 0;       // 是否已经组装出一条等待解析的完整 Frame。
static uint16_t s_read_ptr = 0;         // 软件当前已经消费到的 DMA Circular Buffer 位置。

static uint8_t s_frame_buf[BUFF_SIZE];  // 当前正在组装的 CRSF Frame。

#pragma arm section zidata = "DMA_SAFE_SRAM"

static uint8_t s_dma_rx_buf[DMA_BUF_SIZE]; // USART2 RX DMA Circular Buffer。

#pragma arm section zidata

static RCChannelData_t s_rc_data = {0};     // 最近一次成功解析得到的 RC / Link 状态。
static bool s_has_valid_rc_frame = false;   // 系统启动后是否至少收到过一条有效 RC Channel Frame。
static volatile uint32_t s_last_valid_frame_tick = 0U;      // 最近一条有效 RC Channel Frame 的接收时间。

/*
 * CRSF CRC-8 Lookup Table。
 * 
 * 使用 CRSF CRC-8 Polynomial 预计算得到，
 * 通过查表避免运算时逐 Bit 计算。
 */
static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

/*
 * 将 UART 接收到的单个 Byte 喂入 CRSF 组帧状态机。
 * 
 * Frame 格式：
 * 
 * [Address][Length][Type][Payload...][CRC]
 * 
 * Length 表示从 Type 到 CRC 的总字节数，
 * 因此完整 Frame 长度为 Length + 2。
 */
static void ELRS_StateMachine(uint8_t byte)
{
    switch (s_state)
    {
        case WAIT_SYNC:
        {
            if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                /*
                 * 当前实现只有一块 Frame Buffer。
                 * 上一帧尚未解析时不覆盖它。
                 */
                if (s_frame_ready)
                {
                    break;
                }

                s_idx = 0U;
                s_frame_buf[s_idx++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;

                s_state = READ_LEN;
            }
            break;
        }
        
        case READ_LEN:
        {
            /*
             * CRSF Length 最大为 62，
             * 加上 Address + Lenth 后完整 Frame 最大 64 Byte。
             */
            if (byte > 62)
            {
                s_state = WAIT_SYNC;
                s_idx = 0;
                break;
            }
            
            s_frame_buf[s_idx++] = byte;
            s_frame_len = byte + 2;
            s_state = READ_DATA;
            break;
        }

        case READ_DATA:
        {
            s_frame_buf[s_idx++] = byte;

            if (s_idx == s_frame_len)
            {
                s_state = WAIT_SYNC;
                s_frame_ready = 1;
                s_idx = 0;
            }
            break;
        }

        default:
        {
            s_state = WAIT_SYNC;
            s_idx = 0U;
            break;
        }
    }
}

/*
 * 计算当前 CRSF Frame 的 CRC-8.
 * 
 * CRSF CRC 覆盖范围为：
 * 
 * [Type][Payload...]
 * 
 * 不包含 Address，Length 和 Frame 尾部已有的 CRC Byte。
 */
static uint8_t CRSF_CRC8(void)
{
    uint8_t crc = 0;
    for (int i = 2; i < s_frame_len - 1; i++)
        crc = crc8_table[crc ^ s_frame_buf[i]];

    return crc;
}

/*
 * 解析一条已经完成组帧并通过 CRC 校验的 CRSF Frame。
 * 
 * 当前处理：
 * - RC Channels Packed；
 * - Link Statistics。
 * 
 * 其他合法 Frame 暂时忽略。
 */
static int8_t CRSF_ParseFrame(void)
{
    if (!s_frame_ready)
        return -1;

    s_frame_ready = 0;

    /*
     * CRC Byte 位于完整 Frame 最后一字节。
     */
    if(s_frame_buf[s_frame_len - 1] != CRSF_CRC8())
        return -1;
    
    switch (s_frame_buf[2])
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        {
            /*
             * RC Channels Packed Payload:
             * 16 个 Channel × 11 bit = 176 bit = 22 Byte。
             * 
             * 每个 Channel 的 11 bit 数据连续紧密排列，
             * 不与 Byte Boundary 对齐，因此需要根据 Bit Offset 解包。
             */
            for (int i = 0; i < 16; i++)
            {
                const uint16_t start_bit = (uint16_t)i * 11;
                const uint8_t byte_index = (uint8_t)(start_bit / 8);
                const uint8_t bit_offset = (uint8_t)(start_bit % 8);

                /*
                 * 从当前位置连续拼出足够宽的临时整数，
                 * 再右移到当前 Channel 起始 Bit 并截取最低 11 bit。
                 * 
                 * +3 用于跳过：
                 * Address + Length + Type。
                 */
                const uint32_t raw = 
                    (uint32_t)s_frame_buf[byte_index + 3] | 
                    (uint32_t)s_frame_buf[byte_index + 4] << 8 | 
                    (uint32_t)s_frame_buf[byte_index + 5] << 16;

                s_rc_data.channels[i] = 
                    (uint16_t)((raw >> bit_offset) & 0x7FF);
            }

            /*
             * 只有 CRC 正确的 RC Channel Frame
             * 才证明当前遥控控制链路仍在持续工作。
             */
            s_last_valid_frame_tick = osKernelGetTickCount();
            s_has_valid_rc_frame = true;
            break;
        }

        case CRSF_FRAMETYPE_LINK_STATISTICS:
        {
            /*
             * 当前只提取 Uplink RSSI1，LQ 和 SNR，
             * 其余 ink Statistics 字段暂未使用。
             */
            s_rc_data.rssi = s_frame_buf[3];
            s_rc_data.lq = s_frame_buf[5];
            s_rc_data.snr = (int8_t)s_frame_buf[6];
            break;
        }

        default:
            break;
    }

    return 1;
}

void ELRS_Init(void)
{
    /*
     * USART2 本身由 CubeMX 初始化。
     * 
     * 这里启动 RX DMA 后，DMA 持续循环写入 s_dma_rx_buf；
     * ELRS_Poll() 通过 DMA Current Counter 推算硬件 Write Pointer。
     */
    (void)HAL_UART_Receive_DMA(&huart2, s_dma_rx_buf, DMA_BUF_SIZE);
}

void ELRS_Poll(void)
{
    /*
     * DMA Remaining 表示当前这一轮 Circular DMA 中，
     * 距离本轮传输计数归零还剩多少个 Byte。
     * 
     * 因此：
     * 
     * write_ptr = Buffer Size - Remaining Count
     * 
     * 当 DMA 回绕时再通过取模回到 Buffer 起始位置。
     */
    const uint16_t dma_remaining = 
        (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);

    const uint16_t write_ptr = 
        (uint16_t)(DMA_BUF_SIZE - dma_remaining) % DMA_BUF_SIZE;

    /*
     * 软件 Ready Pointer 追赶 DMA Write Pointer，
     * 期间所有新接收 Byte 依次送入 CRSF Parser。
     */
    while(s_read_ptr != write_ptr)
    {
        ELRS_StateMachine(s_dma_rx_buf[s_read_ptr]);
        s_read_ptr = 
            (uint16_t)(s_read_ptr + 1) % DMA_BUF_SIZE;
    }

    CRSF_ParseFrame();

    /*
     * Link Failsafe 依据最近一条有效 RC Channel Frame 的 Age 判断。
     * 
     * 系统启动后尚未收到任何有效 RC Frame 时，
     * link_ok 始终保持 false。
     */
    uint32_t now = osKernelGetTickCount();

    s_rc_data.link_ok =
        s_has_valid_rc_frame &&
        ((uint32_t)(now - s_last_valid_frame_tick)) <= CRSF_FAILSAFE_TIMEOUT_MS;
}

void ELRS_CopyTo(RCChannelData_t *out)
{
    if(out == NULL)
    {
        return;
    }
    
    *out = s_rc_data;
}
