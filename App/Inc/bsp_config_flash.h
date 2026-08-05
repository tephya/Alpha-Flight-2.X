#ifndef __BSP_CONFIG_FLASH_H
#define __BSP_CONFIG_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_FLASH_PAYLOAD_MAX 40U

typedef enum
{
    CONFIG_FLASH_OK = 0,
    CONFIG_FLASH_NOT_FOUND,
    CONFIG_FLASH_FULL,
    CONFIG_FLASH_BAD_ARGUMENT,
    CONFIG_FLASH_PROGRAM_ERROR,
    CONFIG_FLASH_VERIFY_ERROR,
} ConfigFlashStatus_t;

/**
 * @brief   从追加式配置区读取指定类型和版本的最新有效记录。
 * @param   record_type 记录类型，由上层模块分配。
 * @param   version     记录版本。
 * @param   payload_out 负载输出缓冲区。
 * @param   payload_size    期望负载长度，必须与记录完全一致。
 * @param   sequence_out    可选，返回记录序号
 */
ConfigFlashStatus_t BSP_ConfigFlash_LoadLatest(uint16_t record_type,
                                               uint16_t version,
                                               void *payload_out,
                                               uint16_t payload_size,
                                               uint32_t *sequence_out);

/**
 * @brief   向配置区下一个空Slot追加一条记录，不执行Sector擦除。
 * @note    必须只在Disarmed状态调用。函数最后写magic，掉电时旧记录仍然有效。
 */
ConfigFlashStatus_t BSP_ConfigFlash_Append(uint16_t record_type,
                                           uint16_t version,
                                           const void *payload,
                                           uint16_t payload_size,
                                           uint32_t *sequence_out);

#endif
