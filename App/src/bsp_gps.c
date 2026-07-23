#include "bsp_gps.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>


#define GPS_DMA_BUF_SIZE 256    // 单次burst通常几十到一百多字节，256留足余量
#define NMEA_MAX_LEN 96 // NMEA spec max 82 bytes, 留余量

#define GPS_HOME_MIN_SATELLITES 4       // 起飞点记录成功所需卫星最小数量

static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE];  // DMA直接写入区
static uint8_t gps_proc_buf[GPS_DMA_BUF_SIZE];  // Task读取的快照区，双缓冲
static volatile uint16_t gps_proc_len = 0;
static volatile uint8_t gps_data_ready = 0;     // 1=就绪，0=等待数据传输

static GPS_Data_t gps_data;
static GPS_Home_t gps_home;

/**
 * @brief   挂起DMA传输通道
 */
void GPS_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_dma_buf, GPS_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);     // 半传输中断用不上，关掉减少无谓触发
}

/**
 * @brief   HAL弱函数重写，ISR触发，HAL自动调用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == UART4){
        if(Size > 0 && Size <= GPS_DMA_BUF_SIZE){
            memcpy(gps_proc_buf, gps_dma_buf, Size);        // ISR里只做搬运
            gps_proc_len = Size;
            gps_data_ready = 1;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_dma_buf, GPS_DMA_BUF_SIZE);   // 重新挂起下一包
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

    gps_data.valid = f[2][0];
    gps_data.gps_lat = nmea_to_fixed(f[3], f[4][0]);
    gps_data.gps_lon = nmea_to_fixed(f[5], f[6][0]);
    gps_data.speed_knots = atof(f[7]);
    gps_data.course = atof(f[8]);
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
    int n;

    // 复制一份，因为 split_fields 会修改字符串
    strncpy(buf, line, NMEA_MAX_LEN - 1);
    buf[NMEA_MAX_LEN - 1] = '\0';

    n = split_fields(buf, fields, 20);

    if (strncmp(fields[0], "$GNGGA", 6) == 0)
        parse_gga(fields, n);
    else if (strncmp(fields[0], "$GNRMC", 6) == 0)
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

    uint16_t len = gps_proc_len;
    gps_data_ready = 0;         // 先清标志再处理，避免处理器件被新数据覆盖

    // buf里可能挤着好几条NMEA语句，按'$'...'\n'循环切出来逐条丢给GPS解析函数
    char *p = (char *)gps_proc_buf;
    char *end = p + len;
    while(p < end){
        char *start = memchr(p, '$', end - p);
        if(!start)
            break;
        char *nl = memchr(start, '\n', end - start);
        if(!nl)
            break;              // 不完整的尾巴，丢弃
        *nl = '\0';
        GPS_Parse(start);
        p = nl + 1;
    }
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
