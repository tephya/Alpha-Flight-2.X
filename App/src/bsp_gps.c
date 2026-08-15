#include "bsp_gps.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>


#define GPS_DMA_BUF_SIZE 256    // 单次burst通常几十到一百多字节，256留足余量
#define NMEA_MAX_LEN 96 // NMEA spec max 82 bytes, 留余量

#define GPS_HOME_MIN_SATELLITES 4       // 起飞点记录成功所需卫星最小数量
#define GPS_RMC_UTC_MAX_LEN 16U

#define GPS_BOOT_BAUDRATE 9600U
#define GPS_RUNTIME_BAUDRATE 115200U

#define GPS_RUNTIME_RATE_VERITY_SAMPLES 10U
#define GPS_RUNTIME_RATE_AVG_MIN_MS 70U
#define GPS_RUNTIME_RATE_AVG_MAX_MS 140U
#define GPS_RUNTIME_RATE_VERIFY_TIMEOUT_MS 4000U
#define GPS_RUNTIME_RETRY_MIN_INTERVAL_MS 3000U
#define GPS_RUNTIME_RMC_LOSS_TIMEOUT_MS 2000U
#define GPS_RUNTIME_BAD_WINDOWS_BEFORE_RETRY 2U

#pragma arm section zidata = "DMA_SAFE_SRAM"
static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE];  // DMA直接写入区
#pragma arm section zidata
static uint8_t gps_proc_buf[GPS_DMA_BUF_SIZE];  // Task读取的快照区，双缓冲
static uint8_t gps_parse_buf[GPS_DMA_BUF_SIZE];
static volatile uint16_t gps_proc_len = 0U;
static volatile uint8_t gps_data_ready = 0U;     // 1=就绪，0=等待数据传输

static char s_nmea_line[NMEA_MAX_LEN];
static uint16_t s_nmea_line_len;
static uint8_t s_nmea_collecting;
static char s_last_rmc_utc[GPS_RMC_UTC_MAX_LEN];

static GPS_Data_t gps_data;
static GPS_Home_t gps_home;
static uint32_t s_gps_runtime_verify_started_ms;
static uint32_t s_gps_runtime_last_config_attempt_ms;
static uint32_t s_gps_runtime_last_checked_sequence;
static uint32_t s_gps_rumtime_period_sum_ms;
static uint8_t s_gps_runtime_period_sample_count;
static uint8_t s_gps_runtime_bad_window_count;
static uint8_t s_gps_runtime_rate_verifed;
static uint8_t s_gps_runtime_reconfigure_requested;

static const uint8_t s_gps_set_baud_115200[] = "$PCAS01,5*19\r\n";
static const uint8_t s_gps_set_nmea_gga_rmc_only[] = "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0*32\r\n";
static const uint8_t s_gps_set_rate_10hz[] = "$PCAS02,100*1E\r\n";

static bool GPS_SetUartBaudrate(uint32_t baudrate)
{
    huart4.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart4) == HAL_OK;
}

static bool GPS_SendCommand(const uint8_t *command, uint16_t length)
{
    if(command == NULL || length == 0U)
        return false;

    return HAL_UART_Transmit(&huart4,
                             (uint8_t *)command,
                             length,
                             100U) == HAL_OK;
}

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

/**
 * @brief   从GPS可能处于9600或115200两种状态出发，统一配置到115200、10Hz。
 * @note    PCAS命令没有可靠ACK，因此HAL发送成功只代表本机发送完成；
 *          最终是否成功必须由后续RMC实际周期验证。
 */
static bool GPS_ConfigureRuntime(void)
{
    bool local_operation_ok = true;

    /* 防止UART重初始化与RX回调并发 */
    (void)HAL_UART_AbortReceive(&huart4);
    GPS_ResetReceiveState();

    /* 先在9600发送一次切换命令：GPS若仍为出厂波特率，会切到1115200；
     * GPS若本来就在115200，只会忽略这段错误波特率数据 */
    if(!GPS_SetUartBaudrate(GPS_BOOT_BAUDRATE))
        local_operation_ok = false;

    __HAL_UART_CLEAR_OREFLAG(&huart4);
    if(!GPS_SendCommand(s_gps_set_baud_115200,
                        sizeof(s_gps_set_baud_115200) - 1U))
    {
        local_operation_ok = false;
    }

    osDelay(100U);

    /* MCU切到115200后再次发送波特率命令：兼容GPS上电时已经处于115200的情况，
     * 同时保证失败重试包含波特率配置，而不只是重发语句和频率配置。 */
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

    GPS_ResetReceiveState();
    if(!GPS_StartReceiveDma())
        return false;

    return local_operation_ok;
}

/**
 * @brief   挂起DMA传输通道
 */
void GPS_Init(void)
{
    osDelay(2000U);

    const uint32_t now_ms = HAL_GetTick();
    s_gps_runtime_last_config_attempt_ms = now_ms;

    __HAL_UART_CLEAR_OREFLAG(&huart4);

    (void)GPS_ConfigureRuntime();
    GPS_ResetRumtimeRateVerification(now_ms);
}

/**
 * @brief   HAL弱函数重写，ISR触发，HAL自动调用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == UART4)
    {
        if(Size > 0 && Size <= GPS_DMA_BUF_SIZE)
        {
            const uint16_t used = gps_proc_len;
            const uint16_t free_size = GPS_DMA_BUF_SIZE - used;
            const uint16_t copy_size =
                (Size < free_size) ? Size : free_size;
            
            if(copy_size > 0U)
            {
                memcpy(&gps_proc_buf[used], gps_dma_buf, copy_size); // ISR里只做搬运
                gps_proc_len = used + copy_size;
                gps_data_ready = 1U;
            }    

        }
        // 重新挂起下一包
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_dma_buf, GPS_DMA_BUF_SIZE) == HAL_OK)
        {
            // 每次重启DMA后都要重新关闭HT中断
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
        }
    }
}

/**
 * @brief   将 NMEA 格式坐标转换为定点整数经纬度(单位：1e-7°)
 * @note    全程整数运算，避免float精度损失；
 *          纬度范围约为±90°；经度范围约为±180°；
 *          1e-7°单位下，int32_t 完全够用
 * @param   field 指向坐标字符串的指针
 * @param   direction   半球方向('N', 'S', 'E', 'W')
 * @retval  定点整数坐标，单位1e-7°(南半球和西半球为负值)
 */
static int32_t nmea_to_fixed(const char *field, char direction)
{
    const char *dot = strchr(field, '.');
    if (!dot)
        return 0; // 格式异常保护

    // 解析小数点前的整数部分：ddmm(纬度)或dddmm(经度)
    int32_t int_part = 0;
    const char *p = field;
    while (p < dot)
    {
        int_part = int_part * 10 + (*p - '0');
        p++;
    }

    // 解析小数点后的分数部分，固定处理到5位(0.00001分 ≈ 1.8cm精度)
    int32_t frac_part = 0;
    int frac_digits = 0;
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

    // 拆分度和分：int_part的末两位是分的整数部分，其余是度
    int32_t deg = int_part / 100;
    int32_t minInt = int_part % 100; // 分的整数部分(0~59)

    // min_scaled：分值，放大1e5倍(单位：1e-5分)，避免小数运算
    int64_t min_scaled = (int64_t)minInt * 100000 + frac_part;

    /*十进制度 = deg + min/60，换算到1e-7°定点表示
      用int64_t中间变量防止乘法溢出int32_t范围 */
    int64_t deg_e7 = (int64_t)deg * 10000000LL + (min_scaled * 10000000LL) / (60LL * 100000LL);

    int32_t result = (int32_t)deg_e7;
    if (direction == 'S' || direction == 'W')
        result = -result;

    return result;
}

/**
 * @brief  解析 $GNGGA 定位数据帧
 * @note   提取经纬度、定位质量、卫星数量、HDOP 及海拔信息。
 * @param  f 经过 split_fields 切分后的字符串指针数组
 * @param  n 切分得到的字段总数
 * @retval None
 */
static void parse_gga(char **f, int n)
{
    if (n < 15)
        return;

    /* f[0] $GNGGA
       f[1] 时间
       f[2] 维度
       f[3] N/S
       f[4] 经度
       f[5] E/W
       f[6] 定位质量
       f[7] 卫星数
       f[8] HDOP
       f[9] 海拔
       f[10] 海拔单位M */

    gps_data.gps_lat = nmea_to_fixed(f[2], f[3][0]);
    gps_data.gps_lon = nmea_to_fixed(f[4], f[5][0]);
    gps_data.gps_fix_type = atoi(f[6]);
    gps_data.gps_satellites = atoi(f[7]);
    gps_data.gps_hdop = atof(f[8]);
    gps_data.gps_altitude_m = atof(f[9]);
    gps_data.gga_last_update_ms = HAL_GetTick();
}

/**
 * @brief  解析 $GNRMC 推荐定位数据帧
 * @note   提取经纬度、数据有效性、地面速度、航向及日期信息。
 * @param  f 经过 split_fields 切分后的字符串指针数组
 * @param  n 切分得到的字段总数
 * @retval None
 */
static void parse_rmc(char **f, int n)
{
    if (n < 13)
        return;

    /* f[0] $GNRMC
       f[1] 时间
       f[2] 状态 A/V
       f[3] 维度
       f[4] N/S
       f[5] 经度
       f[6] E/W
       f[7] 速度（节）
       f[8] 航向（度）
       f[9] 日期 */

    // 不把同一UTC历元的重复RMC当成新速度测量
    if ((f[1] == NULL || (f[1][0] == '\0')))
        return;

    if(strncmp(f[1], s_last_rmc_utc, sizeof(s_last_rmc_utc)) == 0)
        return;

    strncpy(s_last_rmc_utc, f[1], sizeof(s_last_rmc_utc) - 1U);
    s_last_rmc_utc[sizeof(s_last_rmc_utc) - 1U] = '\0';

    gps_data.valid = f[2][0];
    gps_data.gps_lat = nmea_to_fixed(f[3], f[4][0]);
    gps_data.gps_lon = nmea_to_fixed(f[5], f[6][0]);
    gps_data.speed_knots = atof(f[7]);
    gps_data.course = atof(f[8]);

    const uint32_t now = HAL_GetTick();
    if(gps_data.rmc_sequence != 0U)
    {
        const uint32_t period = now - gps_data.rmc_last_update_ms;
        gps_data.rmc_period_ms =
            (period > 65535U) ? 65535U : (uint16_t)period;
    }

    gps_data.rmc_last_update_ms = now;
    gps_data.rmc_sequence++;
    if(gps_data.rmc_sequence == 0U)
        gps_data.rmc_sequence = 1U;
}

/**
 * @brief  按逗号切分 NMEA 字段，并去除校验和
 * @note   直接修改输入字符串，将 ',' 和 '*' 替换为 '\0'
 * @param  line 指向 NMEA 数据行的字符串指针
 * @param  fields 用于存储各字段指针的数组
 * @param  max_fields fields 数组的最大容量
 * @retval 切分得到的字段总数
 */
static int16_t split_fields(char *line, char **fields, int max_fields)
{
    int count = 0;
    fields[count++] = line;

    while (*line && count < max_fields)
    {
        if (*line == '*') // 校验和起始，在此处截断
        {
            *line = '\0';
            break;
        }
        if (*line == ',')
        {
            *line = '\0'; // 逗号替换为\0，前一个字段自然终止
            fields[count++] = line + 1;
        }
        line++;
    }
    return count;
}

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

static bool GPS_NmeaChecksumOk(const char *line)
{
    if(line == NULL || line[0] != '$')
        return false;

    const char *star = strchr(line, '*');
    if(star == NULL || star[1] == '\0' || star[2] == '\0')
        return false;

    uint8_t checksum = 0U;
    for (const char *p = line + 1; p < star; p++)
        checksum ^= (uint8_t)*p;

    const int8_t high = GPS_HexValue(star[1]);
    const int8_t low = GPS_HexValue(star[2]);
    if(high < 0 || low < 0)
        return false;

    return checksum == (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
}

/**
 * @brief  对完整的 NMEA 帧进行识别与分发解析
 * @note   内部创建缓冲副本以防止原字符串被切分函数破坏
 * @param  line 指向完整 NMEA 字符串的指针
 * @retval None
 */
static void GPS_Parse(const char *line)
{
    char buf[NMEA_MAX_LEN];
    char *fields[20];

    if(!GPS_NmeaChecksumOk(line))
        return;

    // 复制一份，因为 split_fields 会修改字符串
    strncpy(buf, line, NMEA_MAX_LEN - 1U);
    buf[NMEA_MAX_LEN - 1U] = '\0';

    const int n = split_fields(buf, fields, 20);
    if(n <= 0 || strlen(fields[0]) < 6U)
        return;

    if (strcmp(&fields[0][3], "GGA") == 0)
        parse_gga(fields, n);
    else if (strcmp(&fields[0][3], "RMC") == 0)
        parse_rmc(fields, n);
}

/**
 * @brief   从缓冲区提取一至若干条NMEA帧
 * @note    gps_data里保存的是最新一条完整的ggc/rmc NMEA帧数据
 */
void GPS_Poll(void)
{
    if(!gps_data_ready)
        return;

    /* 生成稳定快照，防止UART ISR在Task解析gps_proc_buf期间覆盖它。
     * 临界区只复制最多256B，不再关中断状态下做字符串解析。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint16_t len = gps_proc_len;
    memcpy(gps_parse_buf, gps_proc_buf, len);

    gps_proc_len = 0U;
    gps_data_ready = 0U;

    if(primask == 0U)
        __enable_irq();

    for (uint16_t i = 0U; i < len; i++)
    {
        const char ch = (char)gps_parse_buf[i];

        if(ch == '$')
        {
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
            // 超长或损坏直接丢弃，等待下一个'$'重新同步
            s_nmea_collecting = 0U;
            s_nmea_line_len = 0U;
        }
    }
}

void GPS_RuntimeService(bool allow_reconfigure)
{
    const uint32_t now_ms = HAL_GetTick();

    if(gps_data.rmc_sequence != 0U &&
        gps_data.rmc_sequence != s_gps_runtime_last_checked_sequence)
    {
        s_gps_runtime_last_checked_sequence = gps_data.rmc_sequence;

        if(gps_data.rmc_period_ms > 0U)
        {
            s_gps_rumtime_period_sum_ms += gps_data.rmc_period_ms;

            if(s_gps_runtime_period_sample_count <
                GPS_RUNTIME_RATE_VERITY_SAMPLES)
            {
                s_gps_runtime_period_sample_count++;
            }

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
                    s_gps_runtime_reconfigure_requested = 1U;
                }
            }
        }
    }

    if(s_gps_runtime_rate_verifed != 0U &&
        gps_data.rmc_last_update_ms != 0U &&
        (uint32_t)(now_ms - gps_data.rmc_last_update_ms) >
            GPS_RUNTIME_RMC_LOSS_TIMEOUT_MS)
    {
        s_gps_runtime_rate_verifed = 0U;
        s_gps_runtime_reconfigure_requested = 1U;
    }

    if(s_gps_runtime_rate_verifed == 0U &&
        s_gps_runtime_reconfigure_requested == 0U &&
        (uint32_t)(now_ms - s_gps_runtime_verify_started_ms) >=
            GPS_RUNTIME_RATE_VERIFY_TIMEOUT_MS)
    {
        s_gps_runtime_reconfigure_requested = 1U;
    }

    if(!allow_reconfigure ||
        s_gps_runtime_reconfigure_requested == 0U ||
        (uint32_t)(now_ms - s_gps_runtime_last_config_attempt_ms) <
            GPS_RUNTIME_RETRY_MIN_INTERVAL_MS)
    {
        return;
    }

    /* 飞行中调用方传false，因此UART/DMA重配置只会发生在Disarmed。 */
    s_gps_runtime_last_config_attempt_ms = now_ms;
    (void)GPS_ConfigureRuntime();
    GPS_ResetRumtimeRateVerification(now_ms);
}

/**
 * @brief   把最近读到的数据拷贝一份给调用方
 * @param   out 调用方提供的接收结构体指针
 */
void GPS_CopyDataTo(GPS_Data_t *out)
{
    *out = gps_data;
}


/**
 * @brief   尝试将当前GPS数据记录为返航点(Home)
 * @note    基于当前已解析的gps_data，不主动触发新的数据读取；
 *          调用方需自行保证在调用前GPS数据已经过Task_Nav周期性轮询更新
 * @retval  true=成功记录，false=定位质量不足，未记录
 */
bool GPS_SetHome(void)
{
    if(gps_data.gps_fix_type >= 1 && gps_data.gps_satellites >= GPS_HOME_MIN_SATELLITES){
        gps_home.altitude = gps_data.gps_altitude_m;
        gps_home.home_lat = gps_data.gps_lat;
        gps_home.home_lon = gps_data.gps_lon;
        return true;
    }
    return false;
}

/**
 * @brief   把起飞点数据拷贝一份给调用方
 * @param   out 调用方提供的接收结构体指针
 */
void GPS_CopyHomeTo(GPS_Home_t *out)
{
    *out = gps_home;
}
