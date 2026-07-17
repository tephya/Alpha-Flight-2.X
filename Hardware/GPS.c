#include "gps.h"

#include <string.h>
#include <stdlib.h>

/* Ring buffer (size 必须 2 的幂，便于用 mask 代替取模) */
#define RB_SIZE  256
#define RB_MASK  (RB_SIZE - 1)
#define NMEA_MAX_LEN  96       /* NMEA spec max 82 bytes, 留余量 */
#define Set_Home	  1
#define To_Be_SetHome 0
#define ReadOK		  1
#define ReadError	  0

static volatile uint8_t  rx_buf[RB_SIZE];
static volatile uint16_t rx_head = 0;   /* ISR writes  */
static volatile uint16_t rx_tail = 0;   /* main reads  */

/* Frame assembly state (only touched by main loop) */
static char     frame_buf[NMEA_MAX_LEN];
static uint16_t frame_idx = 0;
static uint8_t  in_frame  = 0;

static uint8_t ReadState = ReadError;

GPS_Data_t gps_data = {0};
GPS_Home_t gps_home = {0};



/**
  * @brief  初始化 GPS 串口 (UART4) 及其引脚和中断
  * @note   使用 PA0(TX) / PA1(RX)，波特率 9600。开启 RXNE 接收中断。
  * @param  None
  * @retval None
  */
void GPS_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* PA0=TX, PA1=RX, AF8 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = 9600;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &usart);

    nvic.NVIC_IRQChannel                   = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;     /* 不要全 0；给 SysTick 等留余地 */
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    USART_Cmd(UART4, ENABLE);
}

/**
  * @brief  UART4 中断服务函数
  * @note   处理溢出错误 (ORE) 及接收非空中断 (RXNE)，将接收到的字节压入环形缓冲区
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
    /* ORE handling: 必须读 SR 再读 DR 才能清，否则 UART 锁死 */
    if (UART4->SR & USART_SR_ORE) {
        (void)UART4->DR;
        return;
    }

    if (UART4->SR & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(UART4->DR & 0xFF);
        uint16_t next = (rx_head + 1) & RB_MASK;
        if (next != rx_tail) {           /* not full */
            rx_buf[rx_head] = b;
            rx_head = next;
        }
        /* else: 缓冲区满，丢字节（不应该发生：256 字节远大于一帧 NMEA） */
    }
}

/**
  * @brief  从环形缓冲区中提取一个字节
  * @param  b 用于存储读取结果的指针
  * @retval 0: 成功读取, -1: 缓冲区为空
  */
static int rb_get(uint8_t *b)
{
    if (rx_head == rx_tail) return -1;
    *b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & RB_MASK;
    return 0;
}

/**
  * @brief  从环形缓冲区中提取一条完整的 NMEA 帧
  * @note   以 '$' 为起始，'\n' 为结束标识。丢弃帧外字符及超长帧。
  * @param  buf 存放完整 NMEA 字符串的缓冲区
  * @param  max_len 缓冲区的最大允许长度
  * @retval 成功提取的帧长度 (大于0)；若无完整帧则返回 0
  */
int GPS_GetLine(char *buf, int max_len)
{
    uint8_t b;

    while (rb_get(&b) == 0) {
        /* '$' 是 NMEA 帧起始，无条件重置状态 */
        if (b == '$') {
            in_frame  = 1;
            frame_idx = 0;
            frame_buf[frame_idx++] = '$';
            continue;
        }

        if (!in_frame) continue;         /* 帧外字节直接丢 */

        if (frame_idx >= NMEA_MAX_LEN - 1) {
            in_frame  = 0;               /* overflow，丢弃 */
            frame_idx = 0;
            continue;
        }

        frame_buf[frame_idx++] = b;

        if (b == '\n') {                 /* 帧尾 */
            frame_buf[frame_idx] = '\0';
            int len = frame_idx;
            if (len >= max_len) len = max_len - 1;
            memcpy(buf, frame_buf, len);
            buf[len]  = '\0';
            in_frame  = 0;
            frame_idx = 0;
            return len;
        }
    }
    return 0;
}

/**
  * @brief  按逗号切分 NMEA 字段，并去除校验和
  * @note   直接修改输入字符串，将 ',' 和 '*' 替换为 '\0'
  * @param  line 指向 NMEA 数据行的字符串指针
  * @param  fields 用于存储各字段指针的数组
  * @param  max_fields fields 数组的最大容量
  * @retval 切分得到的字段总数
  */
static int split_fields(char *line, char **fields, int max_fields)
{
    int count = 0;
    fields[count++] = line;
    
    while(*line && count < max_fields){
        if(*line == '*'){       /* 校验和起始，截断 */
            *line = '\0';
            break;
        }
        if(*line == ','){
            *line = '\0';       /* 逗号替换为\0，前一个字段自然终止 */
            fields[count++] = line + 1;
        }
        line++;
    }
    return count;
}

/**
  * @brief  将 NMEA 格式坐标转换为十进制经纬度 (Degree Decimal)
  * @note   转换公式: DDD + MM.MMMMM / 60.0。并根据方向附加符号。
  * @param  field 指向坐标字符串的指针 (如 "3211.71113")
  * @param  direction 半球方向 ('N', 'S', 'E', 'W')
  * @retval 转换后的十进制坐标 (南半球和西半球为负值)
  */
static float nmea_to_decimal(const char *field, char direction)
{
    float raw = strtof(field, NULL);           /* 将字符串转为float */
    
    int    deg = (int)(raw / 100.0f);      /* 3211.71113 -> 32deg 0.1171113min */
    float min = raw - deg * 100.0f;
    float result = deg + min / 60.0f;
    
    if(direction == 'S' || direction == 'W') result = -result;      /* N & S > 0, S & W < 0 */
    return result;
}

/**
  * @brief  解析 NMEA 时间字段
  * @note   输入格式应为 HHMMSS (如 "094443.000")
  * @param  field 指向时间字符串的指针
  * @retval None
  */
static void parse_time(const char *field){
    if(field[0] == '\0') return;
    /* "094443.000 -> 09, 44, 43" */
    gps_data.hour = (field[0] - '0') * 10 + (field[1] - '0');       // '0~9' - '0' = 0~9
    gps_data.min  = (field[2] - '0') * 10 + (field[3] - '0');
    gps_data.sec  = (field[4] - '0') * 10 + (field[5] - '0');
}

/**
  * @brief  解析 NMEA 日期字段
  * @note   输入格式应为 DDMMYY (如 "250626")
  * @param  field 指向日期字符串的指针
  * @retval None
  */
static void parse_date(const char *field)
{
    if(field[0] == '\0') return;
    /* "250626" -> 25日，06月，2026年 */
    gps_data.day   = (field[0] - '0') * 10 + (field[1] - '0');
    gps_data.month = (field[2] - '0') * 10 + (field[3] - '0');
    gps_data.year  = 2000 + (field[4] - '0') * 10 + (field[5] - '0');
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
    if(n < 15) return;
    
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
       
    parse_time(f[1]);
    gps_data.latitude   = nmea_to_decimal(f[2], f[3][0]);
    gps_data.longitude  = nmea_to_decimal(f[4], f[5][0]);
    gps_data.fix        = atoi(f[6]);
    gps_data.satellites = atoi(f[7]);
    gps_data.hdop       = atof(f[8]);
    gps_data.altitude   = atof(f[9]);
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
    if(n < 13) return;
    
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
     
    parse_time(f[1]);
    gps_data.valid      = f[2][0];
    gps_data.latitude   = nmea_to_decimal(f[3], f[4][0]);
    gps_data.longitude  = nmea_to_decimal(f[5], f[6][0]);
    gps_data.speed_knots = atof(f[7]);
    gps_data.course     = atof(f[8]);
    parse_date(f[9]);
}

/**
  * @brief  对完整的 NMEA 帧进行识别与分发解析
  * @note   内部创建缓冲副本以防止原字符串被切分函数破坏
  * @param  line 指向完整 NMEA 字符串的指针
  * @retval None
  */
void GPS_Parse(const char *line){
    char buf[NMEA_MAX_LEN];
    char *fields[20];
    int n;
    
    /* 复制一份，因为 split_fields 会修改字符串 */
    strncpy(buf, line, NMEA_MAX_LEN - 1);
    buf[NMEA_MAX_LEN - 1] = '\0';
    
    n = split_fields(buf, fields, 20);
    
    if(strncmp(fields[0], "$GNGGA", 6) == 0)
        parse_gga(fields, n);
    else if(strncmp(fields[0], "$GNRMC", 6) == 0)
        parse_rmc(fields, n);
}

/**
  * @brief  轮询读取并解析所有就绪的 GPS 数据帧
  * @note   更新全局读取状态标志位 ReadState
  * @param  None
  * @retval None
  */
void GPS_Poll(void)
{
    char nmea[NMEA_MAX_LEN];
    while(GPS_GetLine(nmea, NMEA_MAX_LEN)){
        GPS_Parse(nmea);
        ReadState = ReadOK;
    }
}

/**
  * @brief  记录当前位置为返航点 (Home)
  * @note   触发条件: 读取正常、具备定位信息且卫星数量 >= 4
  * @param  None
  * @retval None
  */
void GPS_SetHome(void)
{
    GPS_Poll();
    if(ReadState == ReadOK && gps_data.fix >= 1 && gps_data.satellites >= 4){
        gps_home.altitude = gps_data.altitude;
        gps_home.latitude = gps_data.latitude;
        gps_home.longitude = gps_data.longitude;
        gps_home.valid = Set_Home;
    }
    else
        gps_home.valid = To_Be_SetHome;
}