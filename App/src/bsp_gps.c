/**
 * @file    bsp_gps.c
 * @brief   GPS UART DMA 接收，NMEA 解析及运行参数自检实现。
 */

#include "bsp_gps.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>

/*
 * GPS UART Receive-to-Idle DMA Buffer。
 * 
 * 单次 IDLE Burst 通常明显小于该长度；
 * Task 侧 Processing Buffer 使用相同容量累计 ISR 搬运的数据。
 */
#define GPS_DMA_BUF_SIZE 256
#define NMEA_MAX_LEN 96     // 单条 NMEA Sentence 最大本地存储长度，Byte。

#define GPS_HOME_MIN_SATELLITES 4   // 建立 Home 所要求的最少定位卫星数量。
#define GPS_RMC_UTC_MAX_LEN 16U     // 用于识别重复 RMC UTC 字段的最大保存长度。

#define GPS_BOOT_BAUDRATE 9600U         // GPS 上电默认波特率
#define GPS_RUNTIME_BAUDRATE 115200U    // GPS 目标运行波特率

/*
 * GPS Runtime Rate Verification 参数。
 * 
 * 通过连续统计 RMC 实际周期判断模块是否真正工作在目标更新率，
 * 而不是只根据 PCAS Command 是否成功发送来判断配置结果。
 */
#define GPS_RUNTIME_RATE_VERITY_SAMPLES 10U
#define GPS_RUNTIME_RATE_AVG_MIN_MS 70U
#define GPS_RUNTIME_RATE_AVG_MAX_MS 140U
#define GPS_RUNTIME_RATE_VERIFY_TIMEOUT_MS 4000U
#define GPS_RUNTIME_RETRY_MIN_INTERVAL_MS 3000U
#define GPS_RUNTIME_RMC_LOSS_TIMEOUT_MS 2000U
#define GPS_RUNTIME_BAD_WINDOWS_BEFORE_RETRY 2U

#pragma arm section zidata = "DMA_SAFE_SRAM"

static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE];  // UART4 DMA 直接写入区域。

#pragma arm section zidata

/*
 * ISR / Task 两级接收缓冲：
 * 
 * gps_proc_buf:
 * UART ISR 将每次 Receive-to-Idle 得到的数据快速追加到这里。
 * 
 * gps_parse_buf:
 * GPS_Poll() 在短临界区取得 gps_proc_buf 的稳定快照，
 * 随后在 Task Context 中执行字符串解析。
 */
static uint8_t gps_proc_buf[GPS_DMA_BUF_SIZE];
static uint8_t gps_parse_buf[GPS_DMA_BUF_SIZE];

static volatile uint16_t gps_proc_len = 0U;      // gps_proc_buf 当前已经累计的有效字节数。
static volatile uint8_t gps_data_ready = 0U;     // gps_proc_buf 中是否存在等待 Task 处理的数据。

/* 跨 DMA Burst 保持的 NMEA Sentence 组帧状态。 */
static char s_nmea_line[NMEA_MAX_LEN];
static uint16_t s_nmea_line_len;
static uint8_t s_nmea_collecting;

/* 最近一次已接受的 RMC UTC，用于过滤同一历元的重复 Sentence。 */
static char s_last_rmc_utc[GPS_RMC_UTC_MAX_LEN];

static GPS_Data_t gps_data;
static GPS_Home_t gps_home;

/* Runtime RMC Rate Verification 状态。 */
static uint32_t s_gps_runtime_verify_started_ms;
static uint32_t s_gps_runtime_last_config_attempt_ms;
static uint32_t s_gps_runtime_last_checked_sequence;
static uint32_t s_gps_rumtime_period_sum_ms;
static uint8_t s_gps_runtime_period_sample_count;
static uint8_t s_gps_runtime_bad_window_count;
static uint8_t s_gps_runtime_rate_verifed;
static uint8_t s_gps_runtime_reconfigure_requested;

/*
 * GPS PCAS 配置命令：
 *
 * - 切换到 115200 Baud；
 * - 只输出 GGA / RMC；
 * - 设置 NMEA 更新率为 10 Hz。
 */
static const uint8_t s_gps_set_baud_115200[] = "$PCAS01,5*19\r\n";
static const uint8_t s_gps_set_nmea_gga_rmc_only[] = "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0*32\r\n";
static const uint8_t s_gps_set_rate_10hz[] = "$PCAS02,100*1E\r\n";

/* 重新初始化 UART4，使新的 Baud Rate 生效。 */
static bool GPS_SetUartBaudrate(uint32_t baudrate)
{
    huart4.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart4) == HAL_OK;
}

/* 以阻塞 UART TX 发送一条 GPS 配置命令。 */
static bool GPS_SendCommand(const uint8_t *command, uint16_t length)
{
    if(command == NULL || length == 0U)
        return false;

    return HAL_UART_Transmit(&huart4,
                             (uint8_t *)command,
                             length,
                             100U) == HAL_OK;
}

/*
 * 清空 GPS 软件接收与 NMEA 组帧状态。
 * 
 * gps_proc_len / gps_data_ready 同时被 UART ISR 访问，
 * 因此在此临界区统一复位。
 * 
 * 保留进入函数前的 PRIMASK 状态：
 * 如果调用前中断已经关闭，则函数退出时不会擅自重新打开。
 */
static void GPS_ResetReceiveState(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    gps_proc_len = 0U;
    gps_data_ready = 0U;
    s_nmea_line_len = 0U;
    s_nmea_collecting = 0U;
    s_last_rmc_utc[0] = '\0';

    if(primask == 0U)
        __enable_irq();
}

/*
 * 启动一轮 UART4 Receive-to-Idle DMA。
 * 
 * Receive-to-Idle 以 UART IDLE 或 Buffer 满为事件边界；
 * 实际收到的长度由 HAL_UARTEx_RxEventCallback() 的 Size 输出。
 * 
 * 不需要 Half Transfer Event，因此每次启动 DMA 后关闭 HT Interrupt。
 */
static bool GPS_StartReceiveDma(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart4);

    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart4,
                                    gps_dma_buf,
                                    GPS_DMA_BUF_SIZE) != HAL_OK)
    {
        return false;
    }

    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    return true;
}

/* 重新开始一轮 Runtime RMC Rate Verification。 */
static void GPS_ResetRumtimeRateVerification(uint32_t now_ms)
{
    s_gps_runtime_verify_started_ms = now_ms;
    s_gps_runtime_last_checked_sequence = gps_data.rmc_sequence;
    s_gps_rumtime_period_sum_ms = 0U;
    s_gps_runtime_period_sample_count = 0U;
    s_gps_runtime_bad_window_count = 0U;
    s_gps_runtime_rate_verifed = 0U;
    s_gps_runtime_reconfigure_requested = 0U;
}

/*
 * 将 GPS 尽量统一配置到：
 * 
 * UART = 115200 Baud
 * NMEA = GGA + RMC
 * Rate = 10 Hz
 * 
 * GPS 上电时可能仍处于 9600，也可能已经保持在 115200。
 * 因此先尝试在 9600 发送一次 Baud Command，
 * 再将 MCU UART 切换到 115200 后重新发送一次。
 * 
 * PCAS Command 没有可靠 ACK，
 * HAL_UART_Transmit() 成功只能证明 MCU 完成了本地发送，
 * 最终配置是否真正生效仍由后续 RMC 实际周期验证。
 */
static bool GPS_ConfigureRuntime(void)
{
    bool local_operation_ok = true;

    /*
     * UART Baud Rate 重配置前先终止当前 RX DMA，
     * 防止 UART Re-Init 与 RX Callback 并发修改接收状态。
     */
    (void)HAL_UART_AbortReceive(&huart4);

    GPS_ResetReceiveState();

    /*
     * 第一次按 GPS 默认 9600 Baud 发送切换命令。
     * 
     * 如果 GPS 仍处于 9600，会切换到 115200；
     * 
     * 如果 GPS 本来已经处于 115200，则这条按 9600 Baud 发送的命令
     * 无法被 GPS 正确解码，通常会被视为乱码或帧错误而忽略。
     */
    if(!GPS_SetUartBaudrate(GPS_BOOT_BAUDRATE))
        local_operation_ok = false;

    __HAL_UART_CLEAR_OREFLAG(&huart4);

    if(!GPS_SendCommand(s_gps_set_baud_115200,
                        sizeof(s_gps_set_baud_115200) - 1U))
    {
        local_operation_ok = false;
    }

    osDelay(100U);

    /*
     * MCU 切换到目标 115200 Baud 后再次发送 Baud Command。
     * 
     * 这样无论 GPS 始终处于 9600 还是 115200，
     * 后续 NMEA / Rate Command 都能统一在 Baud 下执行。
     */
    if(!GPS_SetUartBaudrate(GPS_RUNTIME_BAUDRATE))
        return false;

    __HAL_UART_CLEAR_OREFLAG(&huart4);

    if(!GPS_SendCommand(s_gps_set_baud_115200,
                        sizeof(s_gps_set_baud_115200) - 1U))
    {
        local_operation_ok = false;
    }

    osDelay(50U);

    if(!GPS_SendCommand(s_gps_set_nmea_gga_rmc_only,
                        sizeof(s_gps_set_nmea_gga_rmc_only) - 1U))
    {
        local_operation_ok = false;
    }

    osDelay(50U);

    if(!GPS_SendCommand(s_gps_set_rate_10hz,
                        sizeof(s_gps_set_rate_10hz) - 1U))
    {
        local_operation_ok = false;
    }

    osDelay(100U);

    /*
     * 配置阶段可能在 UART RX 中留下无效或不完整字节，
     * 正式恢复接收前统一清楚软件组帧状态。
     */
    GPS_ResetReceiveState();

    if(!GPS_StartReceiveDma())
        return false;

    return local_operation_ok;
}

void GPS_Init(void)
{
    /*
     * GPS 模块上电后预留启动时间，
     * 再开始 UART / NMEA Runtime Configuration。
     */
    osDelay(2000U);

    const uint32_t now_ms = HAL_GetTick();
    s_gps_runtime_last_config_attempt_ms = now_ms;

    __HAL_UART_CLEAR_OREFLAG(&huart4);

    (void)GPS_ConfigureRuntime();

    GPS_ResetRumtimeRateVerification(now_ms);
}

/*
 * UART Receive-to-Idle HAL Callback。
 * 
 * ISR Context 中只完成：
 * 
 * 1. 将本轮 DMA 数据追加到 gps_proc_buf；
 * 2. 标记数据 Ready；
 * 3. 重新启动下一轮 Receive-to-Idle DMA。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance != UART4)
    {
        return;
    }

    if (Size > 0 && Size <= GPS_DMA_BUF_SIZE)
    {
        const uint16_t used = gps_proc_len;
        const uint16_t free_size = GPS_DMA_BUF_SIZE - used;
        const uint16_t copy_size =
            (Size < free_size) 
                ? Size 
                : free_size;

        /*
         * Processing Buffer 尚有空间时追加本轮 DMA Burst。
         * 
         * 若 Task 长时间没有消费导致 Buffer 已满，
         * 超出容量的尾部数据会被丢弃；
         * 后续 NMEA Parser 可通过新的 '$' 重新建立 Sentence 同步。
         */
        if (copy_size > 0U)
        {
            memcpy(&gps_proc_buf[used], gps_dma_buf, copy_size);
            gps_proc_len = used + copy_size;
            gps_data_ready = 1U;
        }
    }
    
    /*
     * 当前 DMA Burst 已经结束，
     * 立即挂起下一轮 Receive-to-Idle。
     */
    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart4,
            gps_dma_buf,
            GPS_DMA_BUF_SIZE) == HAL_OK)
    {
        /*
         * HAL 每次重新启动 DMA 后可能重新使能 HT Event，
         * 因此这里再次显式关闭。
         */
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
}

/*
 * 将 NMEA ddmm.mmmmm / dddmm.mmmmm 坐标
 * 转换为 deg × 1e7 定点整数。
 *
 * 例如：
 *
 * 纬度 2232.12345 表示 22 deg + 32.12345 min。
 *
 * 全过程使用整数中间量，避免直接以 float 保存经纬度造成精度损失。
 * 南纬和西经最终转换为负数。
 */
static int32_t nmea_to_fixed(const char *field, char direction)
{
    const char *dot = strchr(field, '.');

    if (dot == NULL)
    {
        return 0;
    }

    /*
     * 读取小数点前的 ddmm / dddmm 部分。
     * 
     * 最后两位属于 Minute Integer Part，
     * 前面的数字属于 Degree。
     */
    int32_t int_part = 0;

    const char *p = field;

    while (p < dot)
    {
        int_part = int_part * 10 + (*p - '0');
        p++;
    }

    /*
     * Minute Fraction 最多保留 5 位，
     * 即以 1e-5 minute 为最小内部单位。
     * 
     * 位数不足时在右侧补 0，使后续整数换算尺度保持一致。
     */
    int32_t frac_part = 0;
    int32_t frac_digits = 0;

    p = dot + 1;

    while (*p >= '0' && *p <= '9' && frac_digits < 5)
    {
        frac_part = frac_part * 10 + (*p - '0');
        p++;
        frac_digits++;
    }

    // 位数不足时补0对齐
    while (frac_digits < 5)
    {
        frac_part *= 10;
        frac_digits++;
    }

    const int32_t deg = int_part / 100;
    const int32_t minInt = int_part % 100; // 分的整数部分(0~59)

    /*
     * Minute 放大 1e5：
     *
     * min_scaled = (Minute Integer × 1e5) + Minute Fraction)
     */
    int64_t min_scaled = (int64_t)minInt * 100000 + frac_part;

    /*
     * Decimal Degree：
     *
     * deg + minute / 60
     *
     * 最终统一转换成 deg × 1e7。
     * 使用 int64_t 中间量避免乘法过程溢出 int32_t。
     */
    const int64_t deg_e7 = 
        (int64_t)deg * 10000000LL + 
        (min_scaled * 10000000LL) / 
            (60LL * 100000LL);

    int32_t result = (int32_t)deg_e7;

    if (direction == 'S' || direction == 'W')
    {
        result = -result;
    }

    return result;
}

/*
 * 解析一条已经完成字段切分的 GGA Sentence。
 *
 * 使用字段：
 *
 * f[2] Latitude
 * f[3] N/S
 * f[4] Longitude
 * f[5] E/W
 * f[6] Fix Quality
 * f[7] Satellites
 * f[8] HDOP
 * f[9] Altitude
 */
static void parse_gga(char **f, int n)
{
    if (n < 15)
        return;

    gps_data.gps_lat = nmea_to_fixed(f[2], f[3][0]);
    gps_data.gps_lon = nmea_to_fixed(f[4], f[5][0]);
    gps_data.gps_fix_type = (uint8_t)atoi(f[6]);
    gps_data.gps_satellites = (uint8_t)atoi(f[7]);
    gps_data.gps_hdop = (float)atof(f[8]);
    gps_data.gps_altitude_m = (float)atof(f[9]);

    /*
     * Freshness 必须记录真实 Sentence 被解析的时刻，
     * 不能使用 Nav Task 后续发布数据的时间替代。
     */
    gps_data.gga_last_update_ms = HAL_GetTick();
}

/*
 * 解析一条已经完成字段切分的 RMC Sentence。
 *
 * 使用字段：
 *
 * f[1] UTC
 * f[2] Status A/V
 * f[3] Latitude
 * f[4] N/S
 * f[5] Longitude
 * f[6] E/W
 * f[7] Ground Speed，knot
 * f[8] Course Over Ground，deg
 *
 * 同一个 UTC 历元可能重复输出，因此只有 UTC 发生变化时
 * 才认为得到了一次新的 RMC Velocity Sample。
 */
static void parse_rmc(char **f, int n)
{
    if (n < 13)
        return;

    // 不把同一UTC历元的重复RMC当成新速度测量
    if (f[1] == NULL ||
        f[1][0] == '\0')
    {
        return;
    }

    if (strncmp(
            f[1],
            s_last_rmc_utc,
            sizeof(s_last_rmc_utc)) == 0)
    {
        return;
    }

    strncpy(
        s_last_rmc_utc,
        f[1],
        sizeof(s_last_rmc_utc) - 1U);

    s_last_rmc_utc[sizeof(s_last_rmc_utc) - 1U] = '\0';

    gps_data.valid = f[2][0];
    gps_data.gps_lat = nmea_to_fixed(f[3], f[4][0]);
    gps_data.gps_lon = nmea_to_fixed(f[5], f[6][0]);
    gps_data.speed_knots = (float)atof(f[7]);
    gps_data.course = (float)atof(f[8]);

    const uint32_t now = HAL_GetTick();

    /*
     * 从第二个 RMC 历元开始才能计算实际 Sample Period。
     * 
     * 超过 uint16_t 可表示范围时做饱和处理，
     * 避免截断后变成一个错误的小周期。
     */
    if(gps_data.rmc_sequence != 0U)
    {
        const uint32_t period = now - gps_data.rmc_last_update_ms;
        gps_data.rmc_period_ms =
            (period > 65535U) ? 65535U : (uint16_t)period;
    }

    gps_data.rmc_last_update_ms = now;

    /*
     * Sequence 0 保留为“尚未收到 RMC”的初始状态，
     * uint32_t 自然回绕后主动跳过 0。
     */
    gps_data.rmc_sequence++;
    if(gps_data.rmc_sequence == 0U)
        gps_data.rmc_sequence = 1U;
}

/*
 * 按 ',' 原地切分 NMEA Sentence。
 *
 * 每遇到一个 ','：
 * - 将其替换为 '\0'，结束前一个字段；
 * - 保存下一个字段的起始地址。
 *
 * 遇到 '*' 时截断正文，不把尾部 Checksum 字符串作为普通字段。
 *
 * 输入字符串会被直接修改。
 */
static int16_t split_fields(char *line, char **fields, int max_fields)
{
    int count = 0;
    fields[count++] = line;

    while (*line && count < max_fields)
    {
        // 校验和起始，在此处截断
        if (*line == '*') 
        {
            *line = '\0';
            break;
        }

        if (*line == ',')
        {
            // 逗号替换为\0，前一个字段自然终止
            *line = '\0'; 
            fields[count++] = line + 1;
        }

        line++;
    }

    return count;
}

/* 将单个 ASCII Hex 字符转换为 0~15。 */
static int8_t GPS_HexValue(char ch)
{
    if(ch >= '0' && ch <= '9')
        return (int8_t)(ch - '0');

    if(ch >= 'A' && ch <= 'F')
        return (int8_t)(ch - 'A' + 10);

    if(ch >= 'a' && ch <= 'f')
        return (int8_t)(ch - 'a' + 10);

    return -1;
}

/*
 * 校验一条完整 NMEA Sentence。
 *
 * NMEA Checksum 为 '$' 与 '*' 之间所有 ASCII Byte 的逐字节 XOR。
 * '*' 后的两个 Hex 字符表示发送端附带的 8-bit Checksum。
 */
static bool GPS_NmeaChecksumOk(const char *line)
{
    if (line == NULL ||
        line[0] != '$')
    {
        return false;
    }

    const char *star = strchr(line, '*');

    if (star == NULL ||
        star[1] == '\0' ||
        star[2] == '\0')
    {
        return false;
    }

    uint8_t checksum = 0U;

    for (const char *p = line + 1; p < star; p++)
        checksum ^= (uint8_t)*p;
        
    const int8_t high = GPS_HexValue(star[1]);
    const int8_t low = GPS_HexValue(star[2]);

    if (high < 0 ||
        low < 0)
    {
        return false;
    }

    const uint8_t expected =
        (uint8_t)(((uint8_t)high << 4U) |
                  (uint8_t)low);

    return checksum == expected;
}

/*
 * 对一条完整 NMEA Sentence 执行：
 *
 * 1. Checksum Verification
 * 2. Field Split
 * 3. Sentence Type Dispatch
 * 
 * split_fields() 会原地修改字符串，
 * 因此先复制到局部 Buffer，再执行字段切分。
 *
 * 比较 fields[0][3...] 而不是整个 "$GNGGA"，
 * 可以兼容不同 Talker ID，例如 GN / GP。
 */
static void GPS_Parse(const char *line)
{
    char buf[NMEA_MAX_LEN];
    char *fields[20];

    if(!GPS_NmeaChecksumOk(line))
        return;

    // 复制一份，因为 split_fields 会修改字符串
    strncpy(
        buf,
        line,
        NMEA_MAX_LEN - 1U);

    buf[NMEA_MAX_LEN - 1U] = '\0';

    const int n = split_fields(buf, fields, 20);

    if(n <= 0 || strlen(fields[0]) < 6U)
        return;

    if (strcmp(&fields[0][3], "GGA") == 0)
        parse_gga(fields, n);
    else if (strcmp(&fields[0][3], "RMC") == 0)
        parse_rmc(fields, n);
}

void GPS_Poll(void)
{
    if(!gps_data_ready)
        return;

    /*
     * 在短临界区内取得 ISR Processing Buffer 的稳定快照。
     * 
     * 临界区内执行最多 GPS_DMA_BUF_SIZE Byte 的 memcpy。
     * NMEA 扫描，Checksum 和字符串全部在重新开中断后进行。
     */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint16_t len = gps_proc_len;
    memcpy(gps_parse_buf, gps_proc_buf, len);

    gps_proc_len = 0U;
    gps_data_ready = 0U;

    if(primask == 0U)
        __enable_irq();

    /*
     * 一次 DMA 接收到的数据不一定是一条完整的 NMEA Sentence：
     * 一条 Sentence 可能跨多个 DMA Burst，也可能一个 Burst 中包含多条 Sentence。
     * 因此需要跨 GPS_Poll() 保存组帧状态。
     *
     * '$'  -> 开始新的 Sentence
     * '\n' -> Sentence 完成并交给 GPS_Parse()
     * '\r' -> 忽略
     */
    for (uint16_t i = 0U; i < len; i++)
    {
        const char ch = (char)gps_parse_buf[i];

        if(ch == '$')
        {
            /*
             * 新 '$' 同时承担重新同步作用。
             * 即使前一条 Sentence 不完整，也直接从这里重新开始。
             */
            s_nmea_collecting = 1U;
            s_nmea_line_len = 0U;
            s_nmea_line[s_nmea_line_len++] = ch;
            continue;
        }

        if(!s_nmea_collecting)
            continue;

        if(ch == '\n')
        {
            s_nmea_line[s_nmea_line_len] = '\0';
            GPS_Parse(s_nmea_line);
            s_nmea_collecting = 0U;
            s_nmea_line_len = 0U;
            continue;
        }

        if(ch == '\r')
            continue;

        if(s_nmea_line_len < (NMEA_MAX_LEN) - 1U)
        {
            s_nmea_line[s_nmea_line_len++] = ch;
        }
        else
        {
            /*
             * 超长或损坏的 Sentence 直接丢弃，
             * 等待下一次 '$' 重新建立 NMEA 同步。
             */
            s_nmea_collecting = 0U;
            s_nmea_line_len = 0U;
        }
    }
}

void GPS_RuntimeService(bool allow_reconfigure)
{
    const uint32_t now_ms = HAL_GetTick();

    /*
     * 只在出现新的 RMC Sequence 时统计 Sample Period，
     * 避免 Nav Task 高频重复调用导致同一 Sample 被重复计入。
     */
    if(gps_data.rmc_sequence != 0U &&
        gps_data.rmc_sequence != s_gps_runtime_last_checked_sequence)
    {
        s_gps_runtime_last_checked_sequence = 
            gps_data.rmc_sequence;

        if(gps_data.rmc_period_ms > 0U)
        {
            s_gps_rumtime_period_sum_ms += 
                gps_data.rmc_period_ms;

            if(s_gps_runtime_period_sample_count <
                GPS_RUNTIME_RATE_VERITY_SAMPLES)
            {
                s_gps_runtime_period_sample_count++;
            }

            /*
             * 收集一个完整 Period Window 后计算平均 RMC 周期，
             * 以实际输出频率验证 10 Hz 配置是否生效。
             */
            if(s_gps_runtime_period_sample_count >= 
                GPS_RUNTIME_RATE_VERITY_SAMPLES)
            {
                const uint32_t average_period_ms =
                    s_gps_rumtime_period_sum_ms /
                    GPS_RUNTIME_RATE_VERITY_SAMPLES;

                const bool rate_is_10hz =
                    average_period_ms >= GPS_RUNTIME_RATE_AVG_MIN_MS &&
                    average_period_ms <= GPS_RUNTIME_RATE_AVG_MAX_MS;

                s_gps_rumtime_period_sum_ms = 0U;
                s_gps_runtime_period_sample_count = 0U;

                if(rate_is_10hz)
                {
                    s_gps_runtime_rate_verifed = 1U;
                    s_gps_runtime_bad_window_count = 0U;
                    s_gps_runtime_reconfigure_requested = 0U;
                }
                else if(s_gps_runtime_rate_verifed != 0U)
                {
                    /*
                     * 已经验证过正常 Rate 后，不因单个异常 Window
                     * 立即重配置，要求连续多个异常 Window 才认为配置失效。
                     */
                    if(s_gps_runtime_bad_window_count <
                        GPS_RUNTIME_BAD_WINDOWS_BEFORE_RETRY)
                    {
                        s_gps_runtime_bad_window_count++;
                    }

                    /* 已验证后要求连续两个异常窗口，避免偶发漏帧触发重配 */
                    if(s_gps_runtime_bad_window_count >=
                        GPS_RUNTIME_BAD_WINDOWS_BEFORE_RETRY)
                    {
                        s_gps_runtime_rate_verifed = 0U;
                        s_gps_runtime_reconfigure_requested = 1U;
                    }
                }
                else
                {
                    /*
                     * 启动阶段首次 Rate Verification 即失败，
                     * 直接请求下一次 Runtime Configuration。
                     */
                    s_gps_runtime_reconfigure_requested = 1U;
                }
            }
        }
    }

    /*
     * 已经确认工作正常后，如果长时间没有新的 RMC，
     * 认为运行数据链可能失效并请求重新配置。
     */
    if(s_gps_runtime_rate_verifed != 0U &&
        gps_data.rmc_last_update_ms != 0U &&
        (uint32_t)(now_ms - gps_data.rmc_last_update_ms) >
            GPS_RUNTIME_RMC_LOSS_TIMEOUT_MS)
    {
        s_gps_runtime_rate_verifed = 0U;
        s_gps_runtime_reconfigure_requested = 1U;
    }

    /*
     * 启动 / 重配后若一直无法在规定时间内验证目标 RMC Rate，
     * 同样请求再次配置。
     */
    if(s_gps_runtime_rate_verifed == 0U &&
        s_gps_runtime_reconfigure_requested == 0U &&
        (uint32_t)(now_ms - s_gps_runtime_verify_started_ms) >=
            GPS_RUNTIME_RATE_VERIFY_TIMEOUT_MS)
    {
        s_gps_runtime_reconfigure_requested = 1U;
    }

    /*
     * 真正执行 Runtime Configuration 前同时满足：
     *
     * - 调用方允许重配置；
     * - 当前确实存在 Reconfigure Request；
     * - 距离上次尝试已经超过最小重试间隔。
     */
    if(!allow_reconfigure ||
        s_gps_runtime_reconfigure_requested == 0U ||
        (uint32_t)(now_ms - s_gps_runtime_last_config_attempt_ms) <
            GPS_RUNTIME_RETRY_MIN_INTERVAL_MS)
    {
        return;
    }

    /*
     * FlightCtrl / Nav 在 Armed 时传入 false，
     * 因此 UART Abort / Re-Init / DMA Restart 只会在 Disarmed 阶段执行。
     */
    s_gps_runtime_last_config_attempt_ms = now_ms;
    (void)GPS_ConfigureRuntime();
    GPS_ResetRumtimeRateVerification(now_ms);
}

void GPS_CopyDataTo(GPS_Data_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = gps_data;
}

bool GPS_SetHome(void)
{
    /*
     * Home 直接使用最近已经解析完成的 GPS Snapshot，
     * 本函数本身不会主动等待或触发新的 GPS Sample。
     */
    if(gps_data.gps_fix_type >= 1 && 
        gps_data.gps_satellites >= 
            GPS_HOME_MIN_SATELLITES)
    {
        gps_home.altitude = gps_data.gps_altitude_m;
        gps_home.home_lat = gps_data.gps_lat;
        gps_home.home_lon = gps_data.gps_lon;
        return true;
    }
    return false;
}


void GPS_CopyHomeTo(GPS_Home_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = gps_home;
}
