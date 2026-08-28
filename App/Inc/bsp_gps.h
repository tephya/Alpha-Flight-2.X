/**
 * @file    bsp_gps.h
 * @brief   GPS NMEA 数据接收，解析及运行配置接口。
 */

#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   GPS 最新解析数据。
 */
typedef struct
{
    uint8_t gps_fix_type;       /**< GGA Fix Quality：0=无定位，1=GPS，2=DGPS。 */
    uint8_t gps_satellites;     /**< 当前参与定位的卫星数量。 */
    uint8_t valid;              /**< RMC Status：'A'=有效，'V'=无效。 */

    int32_t gps_lat; /**< 纬度，deg × 1e7。 */
    int32_t gps_lon; /**< 经度，deg × 1e7。 */

    float gps_altitude_m; /**< GGA 海拔高度，m。 */
    float gps_hdop;       /**< GGA Horizontal Dilution of Precision。 */

    float speed_knots;     /**< RMC Ground Speed，knot。 */
    float course;          /**< RMC Course Over Ground，deg，相对 True North。 */

    /*
     * NMEA Sample 的实际更新时间必须在成功解析对应 Sentence 时记录，
     * 不能由上层 Nav Task 的固定发布周期推断。
     */
    uint32_t gga_last_update_ms; /**< 最近一次有效 GGA 解析时间，ms。 */
    uint32_t rmc_last_update_ms; /**< 最近一次新 RMC 历元解析时间，ms。 */
    uint32_t rmc_sequence;       /**< 每解析到一个新的 RMC UTC 历元后递增。 */
    uint16_t rmc_period_ms;      /**< 最近两个 RMC 历元之间的实际时间间隔，ms。 */
} GPS_Data_t;

/**
 * @brief GPS Home 参考点。
 */
typedef struct
{
    int32_t home_lat;           /**< Home 纬度，deg × 1e7。 */
    int32_t home_lon;           /**< Home 经度，deg × 1e7。 */
    float altitude;             /**< Home 海拔高度，m。 */
} GPS_Home_t;

/**
 * @brief 初始化 GPS 接收与运行参数配置。
 *
 * 等待 GPS 上电稳定后，将模块配置为运行波特率和目标 NMEA 更新率，
 * 随后启动 UART Receive-to-Idle DMA 接收。
 */
void GPS_Init(void);

/**
 * @brief   处理已经由 UART ISR 搬运到软件缓冲区的 NMEA 字节流。
 * 
 * 从连续字节流中提取完整 NMEA Sentence，校验后更新最新 GPS 状态。
 */
void GPS_Poll(void);

/**
 * @brief   监测 GPS 实际 RMC 更新率，并在需要时重新配置模块。
 * 
 * @param[in] allow_reconfigure 是否允许执行 UART / GPS 重配置。
 * 
 * @note    飞行期间应传入 false，避免实时飞行过程中重启 UART/DMA 数据链。
 */
void GPS_RuntimeService(bool allow_reconfigure);

bool GPS_SetHome(void);
void GPS_CopyDataTo(GPS_Data_t *out);
void GPS_CopyHomeTo(GPS_Home_t *out);

#endif
