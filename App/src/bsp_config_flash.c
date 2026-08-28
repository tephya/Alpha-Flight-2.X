/**
 * @file    bsp_config_flash.c
 * @brief   基于 STM32F405 片内 Flash 的追加式配置存储实现。
 */

#include "bsp_config_flash.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

/*
 * Config Flash 使用 STM32F405RTGx Sector 11。
 *
 * 该地址范围必须同时从 Linker / Scatter File 的程序代码区中排除，
 * 防止固件映像与运行时配置数据占用同一片 Flash。
 */
#define CONFIG_FLASH_START_ADDR 0x080E0000UL
#define CONFIG_FLASH_END_ADDR   0x08100000UL

/** 有效 Config Record 的提交标志，对应 ASCII "CFG1"。 */
#define CONFIG_FLASH_MAGIC      0x43464731UL

/** 每个 Config Slot 固定占用 64 Byte。 */
#define CONFIG_FLASH_SLOT_SIZE 64U

/** 配置区可容纳的固定 Slot 数量。 */
#define CONFIG_FLASH_SLOT_COUNT                          \
    ((CONFIG_FLASH_END_ADDR - CONFIG_FLASH_START_ADDR) / \
     CONFIG_FLASH_SLOT_SIZE)

/**
 * @brief Config Flash 中单个 Slot 的固定二进制布局。
 *
 * 每个 Slot 固定为 64 Byte，并包含 Type、Version、Sequence、
 * Payload 与 CRC32。Magic 既用于快速识别有效 Record，
 * 也作为 Append 操作最后写入的 Commit Marker。
 */
typedef struct
{
    uint32_t magic;         /**< Record Commit Marker。 */
    uint16_t record_type;   /**< 上层分配的配置类型。 */
    uint16_t version;       /**< 当前记录格式版本。 */
    uint32_t sequence;      /**< 同类型记录递增序号。 */

    uint16_t payload_size;  /**< 实际 Payload 长度，Byte。 */
    uint16_t reserved0;     /**< 保留字段。 */

    uint8_t payload[CONFIG_FLASH_PAYLOAD_MAX];  /**< 配置 Payload。 */

    uint32_t reserved1;     /**< 保留字段。 */
    uint32_t crc32;         /**< 从 Record 起始到 crc32 前一字节的 CRC-32。 */
} ConfigFlashRecord_t;

/*
 * 编译期检查 ConfigFlashRecord_t 必须严格占用一个完整 Slot。
 *
 * 若后续修改字段导致布局发生变化，直接在编译阶段报错，
 * 防止读写逻辑仍按 64 Byte Slot 操作而破坏 Flash 数据。
 */
typedef char ConfigFlashRecordSizeMustBe64[(sizeof(ConfigFlashRecord_t) == CONFIG_FLASH_SLOT_SIZE) ? 1 : -1];

/*
 * 计算标准反射式 CRC-32。
 *
 * Polynomial = 0xEDB88320
 * Init       = 0xFFFFFFFF
 * RefIn      = true
 * RefOut     = true
 * XorOut     = 0xFFFFFFFF
 *
 * 用于校验 Config Record 除 crc32 字段自身之外的全部内容。
 */
static uint32_t ConfigFlash_Crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= bytes[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            /*
             * 根据当前 CRC 最低位生成全 0 或 全 1 Mask：
             * 
             * LSB = 0 -> mask = 0x00000000
             * LSB = 1 -> mask = 0xFFFFFFFF
             * 
             * 从而避免为每个 Bit 单独写条件分支。
             */
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));

            crc = (crc >> 1U) ^ (0xEDB88320 & mask);
        }
    }

    return ~crc;
}

/* 根据 Slot Index 获取其 Memory-mapped Flash 地址。 */
static const ConfigFlashRecord_t *ConfigFlash_GetSlot(uint32_t index)
{
    return (const ConfigFlashRecord_t *)(CONFIG_FLASH_START_ADDR +
                                         index * CONFIG_FLASH_SLOT_SIZE);
}

/*
 * 检查一个完整 Slot 是否仍处于擦除状态。
 *
 * STM32 Flash 擦除后的 Bit 均为 1，因此完整空 Slot 的所有
 * 32-bit Word 都应为 0xFFFFFFFF。
 *
 * 写入中途掉电产生的 Partial Record 不再属于 Erased Slot，
 * 后续 Append 会跳过它，而不会尝试覆盖已经 Programming 的 Bit。
 */
static bool ConfigFlash_IsSlotErased(const ConfigFlashRecord_t *slot)
{
    const uint32_t *words = (const uint32_t *)slot;

    for (uint32_t i = 0U; i < (CONFIG_FLASH_SLOT_SIZE / sizeof(uint32_t)); i++)
    {
        if(words[i] != 0xFFFFFFFFUL)
            return false;
    }

    return true;
}

/*
 * 检查一个 Slot 是否为当前调用方可使用的有效 Record。
 *
 * 有效条件包括：
 * - Magic 已经正确写入；
 * - Type / Version 与调用方要求一致；
 * - Payload Size 完全一致且不越界；
 * - Record CRC32 校验通过。
 *
 * 因此写入中途掉电、Bit Error 或错误版本的数据都不会被加载。
 */
static bool ConfigFlash_RecordValid(const ConfigFlashRecord_t *slot,
                                    uint16_t record_type,
                                    uint16_t version,
                                    uint16_t payload_size)
{
    if(slot->magic != CONFIG_FLASH_MAGIC ||
        slot->record_type != record_type ||
        slot->version != version ||
        slot->payload_size != payload_size ||
        slot->payload_size > CONFIG_FLASH_PAYLOAD_MAX)
    {
        return false;
    }

    uint32_t crc = ConfigFlash_Crc32(slot,
        (uint32_t)offsetof(ConfigFlashRecord_t, crc32));

    return crc == slot->crc32;
}

ConfigFlashStatus_t BSP_ConfigFlash_LoadLatest(uint16_t record_type,
                                               uint16_t version,
                                               void *payload_out,
                                               uint16_t payload_size,
                                               uint32_t *sequence_out)
{
    if(payload_out == NULL || payload_size == 0U ||
        payload_size > CONFIG_FLASH_PAYLOAD_MAX)
    {
        return CONFIG_FLASH_BAD_ARGUMENT;
    }

    const ConfigFlashRecord_t *latest = NULL;

    /*
     * 扫描整个配置区。
     * 
     * 不依赖 Record 的物理位置判断新旧，
     * 而是从全部 CRC 有效的匹配记录中选择 Sequence 最新的一条。
     */
    for (uint32_t i = 0U; i < CONFIG_FLASH_SLOT_COUNT; i++)
    {
        const ConfigFlashRecord_t *slot = ConfigFlash_GetSlot(i);

        if(!ConfigFlash_RecordValid(slot, record_type, version, payload_size))
            continue;

        /*
         * 使用有符号 Sequence Difference 判断先后关系，
         * 使 uint32_t Sequence 在自然回绕后仍可继续比较，
         * 前提是参与比较的有效记录跨度不超过半个计数范围。
         */
        if(latest == NULL || (int32_t)(slot->sequence - latest->sequence) > 0)
            latest = slot;
    }

    if (latest == NULL)
        return CONFIG_FLASH_NOT_FOUND;

    memcpy(payload_out, latest->payload, payload_size);

    if (sequence_out != NULL)
        *sequence_out = latest->sequence;

    return CONFIG_FLASH_OK;
}

ConfigFlashStatus_t BSP_ConfigFlash_Append(uint16_t record_type,
                                           uint16_t version,
                                           const void *payload,
                                           uint16_t payload_size,
                                           uint32_t *sequence_out)
{
    if(payload == NULL || 
        payload_size == 0U ||
        payload_size > CONFIG_FLASH_PAYLOAD_MAX)
    {
        return CONFIG_FLASH_BAD_ARGUMENT;
    }

    const ConfigFlashRecord_t *free_slot = NULL;

    uint32_t latest_sequence = 0U;
    bool sequence_valid = false;

    /*
     * 一次扫描同时完成两件事：
     * 
     * 1. 找到物理地址最靠前的完整 Erased Slot；
     * 2. 找到同 Type / Version 的最新有效 Sequence。
     */
    for (uint32_t i = 0U; i < CONFIG_FLASH_SLOT_COUNT; i++)
    {
        const ConfigFlashRecord_t *slot = ConfigFlash_GetSlot(i);

        if(free_slot == NULL && ConfigFlash_IsSlotErased(slot))
            free_slot = slot;

        if(ConfigFlash_RecordValid(
                slot,
                record_type,
                version,
                payload_size))
        {
            if(!sequence_valid || (int32_t)(slot->sequence - latest_sequence) > 0)
            {
                latest_sequence = slot->sequence;
                sequence_valid = true;
            }
        }
    }

    if(free_slot == NULL)
    {
        return CONFIG_FLASH_FULL;
    }

    /*
     * 先在 RAM 中构造完整的 Record。
     * 
     * memset() 同时将未使用的 Payload 区域和 Reserved 字段清零，
     * 使 CRC 输入内容确定，不依赖未初始化 RAM。
     */
    ConfigFlashRecord_t record;

    memset(&record, 0, sizeof(record));
    record.magic = CONFIG_FLASH_MAGIC;
    record.record_type = record_type;
    record.version = version;
    record.sequence = sequence_valid ? (latest_sequence + 1U) : 1U;
    record.payload_size = payload_size;
    memcpy(record.payload, payload, payload_size);

    /*
     * CRC 覆盖 Magic 到 crc32 前的全部字段，
     * 包括 Reserved 与未使用的 Payload 空间。
     */
    record.crc32 = ConfigFlash_Crc32(&record, (uint32_t)offsetof(ConfigFlashRecord_t, crc32));

    const uint32_t target_addr = (uint32_t)free_slot;
    const uint32_t *source_words = (const uint32_t *)&record;
    const uint32_t word_count = CONFIG_FLASH_SLOT_SIZE / sizeof(uint32_t);

    if(HAL_FLASH_Unlock() != HAL_OK)
        return CONFIG_FLASH_PROGRAM_ERROR;

    /*
     * 清除前一次 Flash 操作可能留下的状态标志，
     * 避免旧 Error Flag 干扰本次 Program。
     */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    ConfigFlashStatus_t status = CONFIG_FLASH_OK;
    
    /*
     * 先写 Word 1..N，最后写 Word 0。
     * 
     * Word 0 正好是 Magic，因此 Magic 相当于 Record 的 Commit Marker：
     * 
     * - 中途掉电： Magic 保持 Erased / Invalid，Record 不会被加载；
     * - 全部 Payload 与 CRC 写完后：最后写 Magic，Record 才正式生效。
     * 
     * 原有历史记录始终没有被修改，因此掉电不会破坏上一份有效配置。
     */
    for (uint32_t i = 1U; i < word_count; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                target_addr + i * sizeof(uint32_t),
                                source_words[i]) != HAL_OK)
        {
            status = CONFIG_FLASH_PROGRAM_ERROR;
            break;
        }
    }

    if(status == CONFIG_FLASH_OK)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                            target_addr,
                            source_words[0]) != HAL_OK)
        {
            status = CONFIG_FLASH_PROGRAM_ERROR;
        }
    }

    HAL_FLASH_Lock();

    if(status != CONFIG_FLASH_OK)
        return status;

    /*
     * Program 完成后直接从 Memory-mapped Flash 重新读取 Record，
     * 再执行 Magic / Metadata / CRC 全量校验。
     */
    if (!ConfigFlash_RecordValid(
            (const ConfigFlashRecord_t *)target_addr,
            record_type,
            version,
            payload_size))
    {
        return CONFIG_FLASH_VERIFY_ERROR;
    }

    if(sequence_out != NULL)
        *sequence_out = record.sequence;

    return CONFIG_FLASH_OK;
}
