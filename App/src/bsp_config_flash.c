#include "bsp_config_flash.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

/* STM32F405RTGx Sector 11: 必须同时在Keil scatter文件中从代码区排除。 */
#define CONFIG_FLASH_START_ADDR 0x080E0000UL
#define CONFIG_FLASH_END_ADDR   0x08100000UL
#define CONFIG_FLASH_MAGIC      0x43464731UL /* "CFG1" */
#define CONFIG_FLASH_SLOT_SIZE  64U
#define CONFIG_FLASH_SLOT_COUNT \
    ((CONFIG_FLASH_END_ADDR - CONFIG_FLASH_START_ADDR) / CONFIG_FLASH_SLOT_SIZE)

typedef struct
{
    uint32_t magic;
    uint16_t record_type;
    uint16_t version;
    uint32_t sequence;
    uint16_t payload_size;
    uint16_t reserved0;
    uint8_t payload[CONFIG_FLASH_PAYLOAD_MAX];
    uint32_t reserved1;
    uint32_t crc32;
} ConfigFlashRecord_t;

typedef char ConfigFlashRecordsSizeMustBe64[
    (sizeof(ConfigFlashRecord_t) == CONFIG_FLASH_SLOT_SIZE) ? 1 : -1
];

static uint32_t ConfigFlash_Crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;       // 还原回字节指针
    uint32_t crc = 0xFFFFFFFFUL;   // 标准CRC-32规定，寄存器初值必须全为1

    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            /* 如果最低位是1，掩码则为0xFFFFFFFF(signed -1)
             * 如果最低位是0，掩码则为0x00000000(signed 0) */
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));
            crc = (crc >> 1U) ^ (0xEDB88320 & mask);
        }
    }

    return ~crc;    // 标准CRC-32规定，返回前必须按位取反
}

static const ConfigFlashRecord_t *ConfigFlash_GetSlot(uint32_t index)
{
    return (const ConfigFlashRecord_t *)(CONFIG_FLASH_START_ADDR +
                                         index * CONFIG_FLASH_SLOT_SIZE);
}

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

    for (uint32_t i = 0U; i < CONFIG_FLASH_SLOT_COUNT; i++)
    {
        const ConfigFlashRecord_t *slot = ConfigFlash_GetSlot(i);

        if(!ConfigFlash_RecordValid(slot, record_type, version, payload_size))
            continue;

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
    if(payload == NULL || payload_size == 0U ||
        payload_size > CONFIG_FLASH_PAYLOAD_MAX)
    {
        return CONFIG_FLASH_BAD_ARGUMENT;
    }

    const ConfigFlashRecord_t *free_slot = NULL;
    uint32_t latest_sequence = 0U;
    bool sequence_valid = false;

    for (uint32_t i = 0U; i < CONFIG_FLASH_SLOT_COUNT; i++)
    {
        const ConfigFlashRecord_t *slot = ConfigFlash_GetSlot(i);

        if(free_slot == NULL && ConfigFlash_IsSlotErased(slot))
            free_slot = slot;

        if(slot->magic == CONFIG_FLASH_MAGIC &&
            slot->record_type == record_type &&
            slot->version == version &&
            slot->payload_size == payload_size &&
            ConfigFlash_RecordValid(slot,record_type, version,payload_size))
        {
            if(!sequence_valid || (int32_t)(slot->sequence - latest_sequence) > 0)
            {
                latest_sequence = slot->sequence;
                sequence_valid = true;
            }
        }
    }

    if(free_slot == NULL)
        return CONFIG_FLASH_FULL;

    ConfigFlashRecord_t record;
    memset(&record, 0, sizeof(record));
    record.magic = CONFIG_FLASH_MAGIC;
    record.record_type = record_type;
    record.version = version;
    record.sequence = sequence_valid ? (latest_sequence + 1U) : 1U;
    record.payload_size = payload_size;
    memcpy(record.payload, payload, payload_size);
    record.crc32 = ConfigFlash_Crc32(&record, (uint32_t)offsetof(ConfigFlashRecord_t, crc32));

    const uint32_t target_addr = (uint32_t)free_slot;
    const uint32_t *source_words = (const uint32_t *)&record;
    const uint32_t word_count = CONFIG_FLASH_SLOT_SIZE / sizeof(uint32_t);

    if(HAL_FLASH_Unlock() != HAL_OK)
        return CONFIG_FLASH_PROGRAM_ERROR;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    ConfigFlashStatus_t status = CONFIG_FLASH_OK;
    
    /* 先写word1..15，最后写含magic的word0，避免掉电时把半条记录当成筛选。 */
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

    if(!ConfigFlash_RecordValid((const ConfigFlashRecord_t *)target_addr,
                                record_type, version, payload_size))
    {
        return CONFIG_FLASH_VERIFY_ERROR;
    }

    if(sequence_out != NULL)
        *sequence_out = record.sequence;

    return CONFIG_FLASH_OK;
}
