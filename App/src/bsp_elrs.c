#include "bsp_elrs.h"
#include "usart.h"
#include "cmsis_os2.h"

#define BUFF_SIZE 64        // CRSF单帧最大字节(Sync+Len+Data(≤62))
#define DMA_BUF_SIZE 128

/**
 * CRSF失联判定阈值
 * 150Hz Packet Rate —— 标称间隔6.67ms，x10倍留出偶发丢帧/RF干扰的容忍空间
 * TODO：关闭遥控发射端，观察link_ok多久变false来验证，不合适再调
 */
#define CRSF_FAITLSAFE_TIMEOUT_MS 70U

typedef enum
{
    WAIT_SYNC, // 等Sync
    READ_LEN,  // 读帧长
    READ_DATA  // 读数据
} RCState_t;

static RCState_t s_state = WAIT_SYNC;     // 当前状态
static uint8_t s_idx = 0;            // 缓冲区写入位置
static uint8_t s_frame_len = 0;        
static uint8_t s_frame_ready = 0;
static uint16_t s_read_ptr = 0;

static uint8_t s_frame_buf[BUFF_SIZE];
static uint8_t s_dma_rx_buf[DMA_BUF_SIZE];

static RCChannelData_t s_rc_data = {0};
static volatile uint32_t s_last_valid_frame_tick = 0;

/* 8bit CRC8 码表
 * 查表方式速度更快 */
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

static void ELRS_StateMachine(uint8_t byte)
{
    switch (s_state)
    {
    case WAIT_SYNC:                         // 等Sync
        if(byte == 0xC8)
        {
            if(s_frame_ready)       // 上一帧还没被消费，丢弃新帧
                break;
            s_state = READ_LEN;
            s_frame_buf[s_idx++] = 0xC8;
        }
        break;
    
    case READ_LEN:                         // 读len
        if(byte > 62)
        {
            s_state = WAIT_SYNC;
            s_idx = 0;
            break;
        }
        s_state = READ_DATA;
        s_frame_buf[s_idx++] = byte;
        s_frame_len = byte + 2;
        break;

    case READ_DATA:                        // 收数据
        s_frame_buf[s_idx++] = byte;
        if(s_idx == s_frame_len)
        {
            s_state = WAIT_SYNC;
            s_frame_ready = 1;
            s_idx = 0;
        }
        break;

    default:
        return;
    }
}

/**
 * @brief   计算 CRSF 数据帧的 CRC8 校验码
 * @note    CRSF 协议规范：CRC 校验范围仅包含 Type 字节 和 Payload 数据区，
 * 		    不包含 Sync 字节(0) 和 Length 字节(1)。
 * @param   None
 * @retval  计算出的 8 位 CRC 校验值
 */
static uint8_t CRSF_CRC8(void)
{
    uint8_t crc = 0;
    for (int i = 2; i < s_frame_len - 1; i++)
        crc = crc8_table[crc ^ s_frame_buf[i]];

    return crc;
}

/**
 * @brief   解析 CRSF 数据帧
 */
static int8_t CRSF_ParseFrame(void)
{
    uint8_t offset, byte_index, start_bit;
    uint32_t raw;

    if(!s_frame_ready)
        return -1;

    s_frame_ready = 0;

    if(s_frame_buf[s_frame_len - 1] != CRSF_CRC8())
        return -1;          // CRC失败，拒绝
    
    switch (s_frame_buf[2])
    {
    case 0x16:          // 摇杆通道数据
        for (int i = 0; i < 16; i++)
        {
            start_bit = i * 11;
            byte_index = start_bit / 8;
            offset = start_bit % 8;

            raw = (uint32_t)s_frame_buf[byte_index + 3]         // 跳过 Sync + Len + Type (3Bytes)
                | (uint32_t)s_frame_buf[byte_index + 4] << 8 
                | (uint32_t)s_frame_buf[byte_index + 5] << 16;

            s_rc_data.channels[i] = (uint16_t)((raw >> offset) & 0x7FF);
        }
        break;

    case 0x14:          // 链路质量数据
        s_rc_data.rssi = s_frame_buf[3];
        s_rc_data.lq = s_frame_buf[5];
        s_rc_data.snr = (int8_t)s_frame_buf[6];
        break;

    default:
        break;
    }

    s_last_valid_frame_tick = osKernelGetTickCount();
    return 1;
}

/**
 * @brief   启动DMA循环接收
 * @note    USART2本身由CubeMX的MX_USART2_UART_Init()完成
 */
void ELRS_Init(void)
{
    HAL_UART_Receive_DMA(&huart2, s_dma_rx_buf, DMA_BUF_SIZE);
}

/**
 * @brief   轮询读取 DMA 接收环形缓冲区中的新数据
 * @note    通过计算 DMA 硬件剩余传输量得出 write_ptr，与本地 read_ptr 进行追赶。
 * 		    将追赶过程中的新字节逐个喂入 CRSF 协议状态机进行组帧。
 */
void ELRS_Poll(void)
{
    uint16_t counter = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint16_t write_ptr = (DMA_BUF_SIZE - counter) % DMA_BUF_SIZE;

    while(s_read_ptr != write_ptr)
    {
        ELRS_StateMachine(s_dma_rx_buf[s_read_ptr]);
        s_read_ptr = (s_read_ptr + 1) % DMA_BUF_SIZE;
    }

    CRSF_ParseFrame();

    s_rc_data.link_ok =
        (osKernelGetTickCount() - s_last_valid_frame_tick) <= CRSF_FAITLSAFE_TIMEOUT_MS;
}

void ELRS_CopyTo(RCChannelData_t *out)
{
    *out = s_rc_data;
}
