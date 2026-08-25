/**
 * @file    bsp_config_flash.h
 * @brief   片内 Flash 追加式配置存储接口。
 */

#ifndef __BSP_CONFIG_FLASH_H
#define __BSP_CONFIG_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/** 单条配置允许保存的最大 Payload 长度，Byte。 */
#define CONFIG_FLASH_PAYLOAD_MAX 40U

/**
 * @brief   Config Flash 操作状态。
 */
typedef enum
{
    CONFIG_FLASH_OK = 0,        /**< 操作成功。 */
    CONFIG_FLASH_NOT_FOUND,     /**< 未找到指定 Type / Version 的有效记录。 */
    CONFIG_FLASH_FULL,          /**< 配置区已无完整 Erased Slot 可供追加。 */
    CONFIG_FLASH_BAD_ARGUMENT,  /**< 输入参数非法。 */
    CONFIG_FLASH_PROGRAM_ERROR, /**< Flash Program 操作失败。 */
    CONFIG_FLASH_VERIFY_ERROR,  /**< 写入完成后的完整性校验失败。 */
} ConfigFlashStatus_t;

/**
 * @brief 读取指定 Type / Version 的最新有效配置记录。
 *
 * 配置区采用追加式存储，同一类型可以存在多条历史记录；
 * 本函数扫描全部 Slot，并通过 Sequence 选择最新的一条有效记录。
 *
 * @param[in]  record_type   记录类型，由上层模块分配。
 * @param[in]  version       记录格式版本。
 * @param[out] payload_out   Payload 输出缓冲区。
 * @param[in]  payload_size  期望 Payload 长度，必须与记录中的长度完全一致。
 * @param[out] sequence_out  可选，返回所加载记录的 Sequence；不需要时传 NULL。
 *
 * @return CONFIG_FLASH_OK         读取成功。
 * @return CONFIG_FLASH_NOT_FOUND  未找到匹配的有效记录。
 * @return CONFIG_FLASH_BAD_ARGUMENT 参数非法。
 */
ConfigFlashStatus_t BSP_ConfigFlash_LoadLatest(uint16_t record_type,
                                               uint16_t version,
                                               void *payload_out,
                                               uint16_t payload_size,
                                               uint32_t *sequence_out);

/**
 * @brief 向下一个空 Slot 追加一条配置记录。
 *
 * 采用 Append-only 方式写入，不在本函数中执行 Sector Erase。
 * Record 的 Magic 所在 Word 最后写入，作为记录完整提交的标志；
 * 若写入中途掉电，未完成记录不会通过后续有效性校验，
 * 已经存在的旧记录也不会被覆盖。
 *
 * @param[in]  record_type   记录类型，由上层模块分配。
 * @param[in]  version       记录格式版本。
 * @param[in]  payload       待保存 Payload。
 * @param[in]  payload_size  Payload 长度。
 * @param[out] sequence_out  可选，返回新记录的 Sequence；不需要时传 NULL。
 *
 * @return CONFIG_FLASH_OK             写入并校验成功。
 * @return CONFIG_FLASH_FULL           配置区没有可用 Erased Slot。
 * @return CONFIG_FLASH_BAD_ARGUMENT   参数非法。
 * @return CONFIG_FLASH_PROGRAM_ERROR  Flash Program 失败。
 * @return CONFIG_FLASH_VERIFY_ERROR   写入后的 Record 校验失败。
 *
 * @note Flash Program 会造成不可忽略的执行延迟，应仅在 Disarmed 等
 *       非实时控制阶段调用。
 */
ConfigFlashStatus_t BSP_ConfigFlash_Append(uint16_t record_type,
                                           uint16_t version,
                                           const void *payload,
                                           uint16_t payload_size,
                                           uint32_t *sequence_out);

#endif
