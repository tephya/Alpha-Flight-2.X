#ifndef __GPS_H
#define __GPS_H

#include "stm32f4xx.h"

typedef struct {
	// 定位状态
	uint8_t fix;			// 0=无定位， 1=GPS， 2=DGPS
	uint8_t satellites;		// 可用卫星数
	
	// 位置
	double latitude;		// 十进制度，北正南负，纬度
	double longitude;		// 十进制度，东正西负，经度
	float  altitude;		// 海拔，米（GGA）
	float  hdop;			// 水平精度因子（GGA）
	
	// 运动
	float  speed_knots;		// 地速，节（RMC）
	float  course;			// 航向，度（RMC）
	
	// 时间
	uint8_t hour, min, sec;
	uint8_t day, month;
	uint16_t year;
	
	// 有效性
	uint8_t valid;			// RMC的A/V标志，‘A’=有效
} GPS_Data_t;

typedef struct {
	double latitude;		// home 点的纬度
	double longitude;		// home 点的经度
	float  altitude;		// home 点的 海拔高度
	uint8_t	valid;			// home 点是否已记录; 1=已记录， 0=未记录
} GPS_Home_t;


extern GPS_Data_t gps_data;
extern GPS_Home_t gps_home;

void GPS_Init(void);

/* 取一条完整 NMEA 帧到 buf；返回长度（含 \r\n + '\0'），0 表示暂无完整帧 */
int  GPS_GetLine(char *buf, int max_len);
static int split_fields(char *line, char **fields, int max_fields);
static double nmea_to_decimal(const char *field, char direction);
static void parse_time(const char *field);
static void parse_date(const char *field);
static void parse_gga(char **f, int n);
void GPS_Parse(const char *line);
void GPS_Poll(void);

/*===================== Application =====================*/
void GPS_SetHome(void);							// 记录当前位置为 home 点


#endif