/**
 * @file    bsp_blackbox.c
 * @brief   Blackbox 二进制日志双缓冲及 FatFS 写盘实现。
 */

#include "bsp_blackbox.h"
#include "bsp_sdcard.h"
#include "cmsis_os2.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

/*
 * Blackbox RAM Buffer 大小。
 *
 * Control V4 以约 200 Hz 记录时数据率约为 18 KB/s。
 * 4 KB 双缓冲能够吸收一定程度的 SD Card 短时写入停顿，
 * 同时保持每次完整块写入为 512 B Sector 的整数倍。
 */
#define LOG_BUF_SIZE 4096U

/*
 * 以下 Record Struct 即为实际写入文件的二进制布局。
 *
 * 每条记录均以 MAGIC + TYPE 开头，
 * 并使用 Packed Layout 保证 MCU 与离线解析器看到完全一致的字节排列。
 */

/**
 * @brief Motion Record。
 */
typedef __packed struct
{
    uint8_t magic;         /**< BB_FRAME_MAGIC。 */
    uint8_t type;          /**< BB_REC_MOTION。 */
    uint16_t time_ms;      /**< 时间戳，ms。 */
    int16_t angle_cdeg[3]; /**< Roll/Pitch/Yaw，0.01 deg。 */
    uint16_t motor[4];     /**< 四路 Motor Output。 */
    int8_t target_cdeg[3]; /**< Roll/Pitch/Yaw Target，0.01 deg。 */
} BB_MotionRec_t;

/**
 * @brief Arm 状态变化 Record。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
    uint8_t armed;
} BB_ArmChangedRec_t;

/**
 * @brief 无附加 Payload 的 Fault Record。
 *
 * Dual Fault 与 Voltage Fault 通过 Type 区分。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
} BB_FaultRec_t;

/**
 * @brief Active IMU 切换 Record。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint16_t time_ms;
    uint8_t new_active_imu;
} BB_ImuSwitchRec_t;

/**
 * @brief Control V4 完整 Record。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    BB_ControlData_t data;
} BB_ControlRec_t;

/**
 * @brief Navigation V5 完整 Record。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    BB_NavigationData_t data;
} BB_NavigationRec_t;

/*
 * 编译期检查实际日志 Record Size。
 *
 * 修改 Payload 后如果忘记同步协议版本或解析器，
 * 编译期 Size Check 会直接暴露格式变化。
 */

typedef char BB_NavigationRecSizeMustBe76[(sizeof(BB_NavigationRec_t) == 76U) ? 1 : -1];

typedef char BB_ControlRecV4SizeMustBe90[(sizeof(BB_ControlRec_t) == 90U) ? 1 : -1];

/**
 * @brief Mag Calibration 原始采样 Record。
 */
typedef __packed struct
{
    uint8_t magic;
    uint8_t type;
    uint32_t timestamp_cycle;
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
} BB_MagCalibrationRec_t;

typedef char BB_MagCalibrationRecSizeMustBe18[(sizeof(BB_MagCalibrationRec_t) == 18U) ? 1 : -1];

/**
 * @brief 单块 Blackbox Buffer 状态。
 */
typedef enum
{
    BB_BUF_FREE = 0, /**< 当前 Buffer 可供 Producer 使用。 */
    BB_BUF_FILLING,  /**< Producer 正在向该 Buffer 写入 Record。 */
    BB_BUF_READY,    /**< Buffer 已填满，等待 Consumer 写入 SD Card。 */
} BB_BufferState_t;

/**
 * @brief 单块 Blackbox RAM Buffer。
 */
typedef struct
{
    uint8_t data[LOG_BUF_SIZE];      /**< 原始日志字节。 */
    uint16_t pos;                    /**< 当前已使用字节数。 */
    volatile BB_BufferState_t state; /**< Producer / Consumer 共享状态。 */
} BB_Buffer_t;

static FATFS s_fs;
static FIL s_fil;

/*
 * Blackbox 双缓冲放置在指定 SRAM 区域。
 */
#pragma arm section zidata = "DMA_SAFE_SRAM"

static BB_Buffer_t s_buf[2];

#pragma arm section zidata

/** 当前 Producer 正在填充的 Buffer Index。 */
static uint8_t s_fillIndex = 0U;

/** Ready Buffer 通知 Blackbox Consumer 的 Semaphore。 */
static osSemaphoreId_t s_dataReadySem = NULL;

/** File Open / Close 控制请求 EventFlags。 */
static osEventFlagsId_t s_ctrlEvt = NULL;

/** 运行期间是否发生过双缓冲 Overflow。 */
static volatile uint8_t s_overflowFlag = 0U;

/** 运行期间是否发生过 SD Write Error。 */
static volatile uint8_t s_writeErrorFlag = 0U;

/*
 * 发布当前已填满的 Buffer，并切换到另一块 Free Buffer。
 *
 * 双缓冲中只有另一块已经被 Consumer 回收为 Free，
 * Producer 才能继续写入。
 *
 * @return 0  切换成功。
 * @return -1 另一块 Buffer 尚未消费，当前新数据无法继续写入。
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

/*
 * 将一条完整 Record 的原始字节写入双缓冲。
 *
 * Record 可以跨越当前 Buffer 边界：
 * 当前块填满后先 Publish，再继续写入下一块。
 *
 * 若跨边界时无法取得下一块 Free Buffer，则回滚本条 Record
 * 在当前 Buffer 中已经写入的字节，保证日志文件中不会留下残缺 Record。
 *
 * 当前所有 Record Size 均小于 LOG_BUF_SIZE，
 * 因此单条 Record 最多跨越一次 Buffer Boundary。
 */
static int8_t BB_WriteBytes(const uint8_t *src, uint16_t len)
{
    if(src == NULL || len > LOG_BUF_SIZE)
        return -1;

    const uint8_t start_index = s_fillIndex;
    const uint16_t start_pos = s_buf[start_index].pos;
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
            {
                /*
                 * Publish 失败时 Fill Index 尚未切换，
                 * 因此直接恢复进入本条 Record 前的位置。
                 * 
                 * 之前已经完整写入的旧 Record 保持不变，
                 * 下一次写入会从 start_pos 覆盖当前残留字节。
                 */
                s_buf[start_index].pos = start_pos;
                return -1;
            }
        }
    }
    return 0;
}

int8_t BB_Init(void)
{
    char filename[16];
    FILINFO fno;
    
    /*
     * 挂载当前 SD Card FatFS。
     * 这里本身没有执行 BSP_SD_Init()，底层 SD 初始化必须由外部启动流程完成。
     */
	FRESULT res = f_mount(&s_fs, "", 1);
    if(res != FR_OK)
        return -1;

    /*
     * 搜索首个未被占用的 LOSxxx.BIN 文件名。
     */
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

void BB_BufferInit(void)
{
    s_buf[0].pos = 0;
    s_buf[0].state = BB_BUF_FILLING;

    s_buf[1].pos = 0;
    s_buf[1].state = BB_BUF_FREE;

    s_fillIndex = 0;

    /*
     * Buffer Semaphore 只创建一次。
     * 每次创建新日志文件都会重新初始化 Buffer State，
     * 但不能反复创建新的 RTOS Kernel Object，否则会造成资源泄漏。
     */
    if(s_dataReadySem == NULL)
        s_dataReadySem = osSemaphoreNew(2, 0, NULL);

    s_overflowFlag = 0;
    s_writeErrorFlag = 0;
}

void BB_ControlInit(void)
{
    /*
     * 控制 EventFlags 同样只创建一次，
     * 必须在第一次 Request / Poll 之前完成初始化。
     */
    if(s_ctrlEvt == NULL)
        s_ctrlEvt = osEventFlagsNew(NULL);
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
    BB_ImuSwitchRec_t rec = {BB_FRAME_MAGIC, BB_REC_IMU_SWITCH, time_ms, new_active_imu};
    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogControl(const BB_ControlData_t *data)
{
    BB_ControlRec_t rec;

    rec.magic = BB_FRAME_MAGIC;
    rec.type = BB_REC_CONTROL_V4;
    rec.data = *data;

    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogNavigation(const BB_NavigationData_t *data)
{
    if(data == NULL)
        return -1;

    BB_NavigationRec_t rec;
    rec.magic = BB_FRAME_MAGIC;
    rec.type = BB_REC_NAVIGATION_V5;
    rec.data = *data;

    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_LogMagCalibration(uint32_t timestamp_cycle,
                            float mag_x_gauss,
                            float mag_y_gauss,
                            float mag_z_gauss)
{
    BB_MagCalibrationRec_t rec;

    rec.magic = BB_FRAME_MAGIC;
    rec.type = BB_REC_MAG_CAL_SAMPLE;
    rec.timestamp_cycle = timestamp_cycle;
    rec.mag_x_gauss = mag_x_gauss;
    rec.mag_y_gauss = mag_y_gauss;
    rec.mag_z_gauss = mag_z_gauss;

    return BB_WriteBytes((const uint8_t *)&rec, sizeof(rec));
}

int8_t BB_WaitReady(uint32_t timeout_ms)
{
    osStatus_t st = osSemaphoreAcquire(s_dataReadySem, timeout_ms);
    return (st == osOK) ? 0 : -1;
}

int8_t BB_Process(void)
{
    int8_t index = -1;
    
    /*
     * 双缓冲最多同时存在两块 Ready Buffer，
     * 每次调用只消费其中一块。
     */
    if(s_buf[0].state == BB_BUF_READY)
        index = 0;
    else if(s_buf[1].state == BB_BUF_READY)
        index = 1;
    else
        return 0;

    UINT bw;
    FRESULT result = f_write(&s_fil, s_buf[index].data, LOG_BUF_SIZE, &bw);

    if(result == FR_OK && bw == LOG_BUF_SIZE)
    {
        s_buf[index].pos = 0;
        s_buf[index].state = BB_BUF_FREE;
    }
    else
    {
        /*
         * SD Write 失败后丢弃当前块并回收 Buffer，
         * 避免 Consumer 永久卡在无法回收的错误状态。
         * 
         * writeErrorFlag 会保留本次数据完整性已经受损的事实。
         */
        s_writeErrorFlag = 1;

        s_buf[index].pos = 0U;
        s_buf[index].state = BB_BUF_FREE;
    }

    /*
     * 返回 1 表示本次确实消费了一块；
     * 调用方可以继续调用，知道返回 0 为止。
     */
    return 1; 
}

int8_t BB_Close(void)
{
    uint16_t len = s_buf[s_fillIndex].pos;

    /*
     * READY Buffer 应由调用方在进入 BB_Close() 前通过 BB_Process()
     * 全部消费完成；这里只负责当前 Filling Buffer 中不足整块的尾部数据。
     */
    if(len > 0)
    {
        UINT written = 0;
        const FRESULT result = f_write(&s_fil, s_buf[s_fillIndex].data, len, &written);
        
        if(result != FR_OK || written != len)
        {
            f_close(&s_fil);
            return -1;
        }

        s_buf[s_fillIndex].pos = 0;

        /*
         * 将 FatFS Cache 明确同步到存储介质后再关闭文件。
         */
        if (f_sync(&s_fil) != FR_OK)
        {
            (void)f_close(&s_fil);
            return -1;
        }
    }

    return (f_close(&s_fil) == FR_OK) ? 0 : -2;
}

uint8_t BB_GetErrorFlags(void)
{
    uint8_t flags = 0;

    if(s_overflowFlag)
        flags |= (1U << 0);

    if(s_writeErrorFlag)
        flags |= (1U << 1);

    return flags;
}

void BB_RequestClose(void)
{
    if(s_ctrlEvt != NULL)
    {
        (void)osEventFlagsSet(s_ctrlEvt, BB_CTRL_CLOSE_REQ);
    }
}

void BB_RequestNewFile(void)
{
    if(s_ctrlEvt != NULL)
    {
        (void)osEventFlagsSet(s_ctrlEvt, BB_CTRL_NEWFILE_REQ);
    }
}

uint32_t BB_PollControlRequest(uint32_t timeout_ms)
{
    if(s_ctrlEvt == NULL)
    {
        return osFlagsErrorResource;
    }

    return osEventFlagsWait(
        s_ctrlEvt, 
        BB_CTRL_CLOSE_REQ | BB_CTRL_NEWFILE_REQ,
        osFlagsWaitAny, 
        timeout_ms);
}
