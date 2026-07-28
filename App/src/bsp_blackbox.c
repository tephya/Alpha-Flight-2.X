#include "bsp_blackbox.h"
#include "bsp_sdcard.h"
#include "cmsis_os2.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

#define LOG_BUF_SIZE 512U

/*====== 各记录类型的实际大小(含MAGIC + type + 内容), __packed保证紧凑 ======*/
typedef __packed struct
{
    uint8_t magic;      // BB_FRAME_MAGIC，帧同步用
    uint8_t type;       // BB_REC_MOTION
    uint16_t time_ms;
    int16_t angle_cdeg[3];  // roll/pitch/yaw，0.01°定点
    uint16_t motor[4];
    int8_t target_cdeg[3];
} BB_MotionRec_t;       // total = 21Bytes

typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
    uint8_t armed;
} BB_ArmChangedRec_t;   // total = 5Bytes

typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
} BB_FaultRec_t;        // total = 4Bytes, DualFault/VoltageFault共用

typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
    uint8_t new_active_imu;
} BB_ImuSwitchRec_t;    // total = 5Bytes

/*====== 双缓冲消费状态(内部私有) ======*/
typedef enum
{
    BB_BUF_FREE,
    BB_BUF_FILLING,
    BB_BUF_READY,
} BB_BufferState_t;

typedef struct 
{
    uint8_t data[LOG_BUF_SIZE];
    uint16_t pos;
    volatile BB_BufferState_t state;
} BB_Buffer_t;

static FATFS s_fs;
static FIL s_fil;

static BB_Buffer_t s_buf[2];        // 创建双缓冲区
static uint8_t s_fillIndex = 0;
static osSemaphoreId_t s_dataReadySem = NULL;

static volatile uint8_t s_overflowFlag = 0;
static volatile uint8_t s_writeErrorFlag = 0;

/**
 * @brief   把fill_index切到另一块空闲缓冲区，当前块标记READY并唤醒消费者
 * @retval  -1 : 另一块也没被消费完，发生一处(本次数据丢弃)
 *           0 : 成功
 */
static int8_t BB_PublishCurrentBlock(void)
{
    uint8_t next = s_fillIndex ^ 1U;

    if(s_buf[next].state != BB_BUF_FREE)
    {
        s_overflowFlag = 1;
        return -1;
    }

    s_buf[s_fillIndex].state = BB_BUF_READY;
    osSemaphoreRelease(s_dataReadySem);

    s_fillIndex = next;
    s_buf[s_fillIndex].pos = 0;
    s_buf[s_fillIndex].state = BB_BUF_FILLING;

    return 0;
}

/**
 * @brief   把一条记录的原始字节写入当前缓冲区，跨512字节边界自动触发Publish
 */
static int8_t BB_WriteBytes(const uint8_t *src, uint16_t len)
{
    uint16_t remaining = len;
    
    while(remaining > 0)
    {
        uint16_t space = LOG_BUF_SIZE - s_buf[s_fillIndex].pos;
        uint16_t copy_size = (remaining < space) ? remaining : space;

        memcpy(&s_buf[s_fillIndex].data[s_buf[s_fillIndex].pos], src, copy_size);

        s_buf[s_fillIndex].pos += copy_size;
        src += copy_size;
        remaining -= copy_size;

        if(s_buf[s_fillIndex].pos == LOG_BUF_SIZE)
        {
            if(BB_PublishCurrentBlock() != 0)
                return -1;
        }
    }
    return 0;
}

/**
 * @brief   挂载SD卡文件系统 + 创建新日志文件(LOGxxx.BIN)
 * @note    内部调用BSP_SD_Init完成SD卡上电，成功后才尝试挂载FatFS。
 *          必须在调度器启动后、由Task_Blackbox调用(内部有阻塞操作)。
 * @retval  0 : 成功
 *         -1 : SD卡初始化/挂载失败
 *         -2 : 文件创建失败
 */
int8_t BB_Init(void)
{
    char filename[16];
    FILINFO fno;

    if(BSP_SD_Init() != 0)
        return -1;
    
    if(f_mount(&s_fs, "", 1) != FR_OK)
        return -1;

    for (int i = 0; i < 1000; i++)
    {
        snprintf(filename, sizeof(filename), "LOG%03d.BIN", i);
        if(f_stat(filename, &fno) == FR_NO_FILE)
            break;
    }

    if(f_open(&s_fil, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return -2;

    return 0;
}

/**
 * @brief   初始化双缓冲消费者状态，创建“缓冲区就绪”信号量
 * @note    须在BB_Init成功后、Task_Blackbox主循环开始前调用一次
 */
void BB_BufferInit(void)
{
    s_buf[0].pos = 0;
    s_buf[0].state = BB_BUF_FILLING;

    s_buf[1].pos = 0;
    s_buf[1].state = BB_BUF_FREE;

    s_fillIndex = 0;
    s_dataReadySem = osSemaphoreNew(2, 0, NULL);        // 最多两块缓冲同时待处理
    s_overflowFlag = 0;
    s_writeErrorFlag = 0;
}

int8_t BB_LogMotion(uint16_t time_ms, const int16_t angle_cdeg[3],
                    const uint16_t motor[4], const int8_t target_cdeg[3])
{
    BB_MotionRec_t rec;
    rec.magic = BB_FRAME_MAGIC;
    rec.type = BB_REC_MOTION;
    rec.time_ms = time_ms;
    memcpy((void*)rec.angle_cdeg, angle_cdeg, sizeof(rec.angle_cdeg));
    memcpy((void*)rec.motor, motor, sizeof(rec.motor));
    memcpy((void*)rec.target_cdeg, target_cdeg, sizeof(rec.target_cdeg));
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogArmChanged(uint16_t time_ms, uint8_t armed)
{
    BB_ArmChangedRec_t rec = {BB_FRAME_MAGIC, BB_REC_ARM_CHANGED, time_ms, armed};
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogDualFault(uint16_t time_ms)
{
    BB_FaultRec_t rec = {BB_FRAME_MAGIC, BB_REC_DUAL_FAULT, time_ms};
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogVoltageFault(uint16_t time_ms)
{
    BB_FaultRec_t rec = {BB_FRAME_MAGIC, BB_REC_VOLTAGE_FAULT, time_ms};
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogImuSwitch(uint16_t time_ms, uint8_t new_active_imu)
{
    BB_ImuSwitchRec_t rec = {BB_FRAME_MAGIC, BB_REC_IMU_SWITCH, time_ms};
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

/**
 * @brief   阻塞等待某个缓冲区被生产者填满、可以写盘
 * @param   timeout_ms  等待超时
 * @retval  0 : 有数据待写，应调用BB_Process()
 *         -1 : 超时，无数据
 */
int8_t BB_WaitReady(uint32_t timeout_ms)
{
    osStatus_t st = osSemaphoreAcquire(s_dataReadySem, timeout_ms);
    return (st == osOK) ? 0 : -1;
}

/**
 * @brief   消费一块READY状态的缓冲区，写入SD卡，更新缓冲区状态
 * @note    由Task_Blackbox在BB_WaitReady返回0后调用
 */
void BB_Process(void)
{
    int8_t index = -1;
    
    if(s_buf[0].state == BB_BUF_READY)
        index = 0;
    else if(s_buf[1].state == BB_BUF_READY)
        index = 1;
    else
        return;

    UINT bw;
    FRESULT result = f_write(&s_fil, s_buf[index].data, LOG_BUF_SIZE, &bw);

    if(result == FR_OK && bw == LOG_BUF_SIZE)
    {
        s_buf[index].pos = 0;
        s_buf[index].state = BB_BUF_FREE;
    }
    else
    {
        s_writeErrorFlag = 1;
        s_buf[index].state = BB_BUF_FREE;       // 丢弃这一块，避免消费者卡死咋ERROR态无法回收
    }
}

/**
 * @brief   将当前未写满的尾部数据flush到SD卡并关闭文件
 * @retval  0 : 成功
 *         -1 : 写入失败/漏写数据
 *         -2 : 关闭文件失败
 */
int8_t BB_Close(void)
{
    uint16_t len = s_buf[s_fillIndex].pos;

    if(len > 0)
    {
        UINT written = 0;
        FRESULT result = f_write(&s_fil, s_buf[s_fillIndex].data, len, &written);
        if(result != FR_OK || written != len)
        {
            f_close(&s_fil);
            return -1;
        }
        s_buf[s_fillIndex].pos = 0;
        f_sync(&s_fil);
    }

    return (f_close(&s_fil) == FR_OK) ? 0 : -2;
}

/**
 * @brief   查询运行期间是否发生过缓冲区溢出或写入错误
 * @retval  bit0=1: 曾发生缓冲区溢出(两块缓冲区都被占用，丢了数据)
 *          bit1=1: 层发生SD写入错误
 */
uint8_t BB_GetErrorFlags(void)
{
    uint8_t flags = 0;
    if(s_overflowFlag)
        flags |= (1U << 0);
    if(s_writeErrorFlag)
        flags |= (1U << 1);
    return flags;
}
