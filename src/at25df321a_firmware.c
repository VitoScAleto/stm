#include "../inc/at25df321a_firmware.h"
#include "../inc/at25df321a.h"
#include <string.h>
#include <stdlib.h>

#define FIRMWARE_MAGIC             0xDEADBEEF
#define FIRMWARE_TABLE_ADDRESS     0x003F0000u
#define FIRMWARE_DATA_END_ADDRESS  FIRMWARE_TABLE_ADDRESS
#define FIRMWARE_INACTIVE_STATUS   0u
#define FIRMWARE_ACTIVE_STATUS     1u

static FirmwareManager_t firmware_manager;
static bool initialized = false;

inline void delay_us(uint32_t us);

 inline void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000u);
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles) {
    }
}

static uint32_t CalculateCRC32(const uint8_t* data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for(uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(int j = 0; j < 8; j++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return ~crc;
}

static bool EraseRange4K(uint32_t address, uint32_t length) {
    if(length == 0) return true;

    uint32_t start = address & ~(AT25_SECTOR_SIZE - 1u);
    uint32_t end = address + length;
    end = (end + AT25_SECTOR_SIZE - 1u) & ~(AT25_SECTOR_SIZE - 1u);

    for(uint32_t a = start; a < end; a += AT25_SECTOR_SIZE) {
        if(!AT25_SectorErase(a)) return false;
    }
    return true;
}

static bool SaveFirmwareTable(void) {
    if(!EraseRange4K(FIRMWARE_TABLE_ADDRESS, sizeof(FirmwareManager_t))) return false;
    return AT25_WriteBytes(FIRMWARE_TABLE_ADDRESS, (uint8_t*)&firmware_manager, sizeof(FirmwareManager_t));
}

static bool LoadFirmwareTable(void) {
    FirmwareManager_t saved_table;
    AT25_ReadData(FIRMWARE_TABLE_ADDRESS, (uint8_t*)&saved_table, sizeof(FirmwareManager_t));

    if(saved_table.firmware_count > MAX_FIRMWARE_COUNT) {
        memset(&firmware_manager, 0, sizeof(firmware_manager));
        return false;
    }

    firmware_manager = saved_table;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        FirmwareHeader_t header;
        uint32_t saved_status = firmware_manager.firmware_table[i].header.status;

        AT25_ReadData(firmware_manager.firmware_table[i].address, (uint8_t*)&header, sizeof(FirmwareHeader_t));

        if(header.magic == FIRMWARE_MAGIC &&
           firmware_manager.firmware_table[i].address + FIRMWARE_HEADER_SIZE + header.size <= FIRMWARE_DATA_END_ADDRESS) {
            header.status = saved_status;      // active/inactive хранится в таблице, не в секторе данных
            firmware_manager.firmware_table[i].header = header;
            firmware_manager.firmware_table[i].valid = 1;
        } else {
            firmware_manager.firmware_table[i].valid = 0;
        }
    }

    return true;
}
int swd_prepare_target_for_program(uint32_t firmware_size)
{
    swd_gpio_init();
    dwt_init();

    NRST_LOW();
    delay_us(5000);
    NRST_HIGH();
    delay_us(50000);

    if (!swd_init_debug()) {
        return 0;
    }

    flash_unlock_target();
    flash_erase_all_needed(FLASH_BASE_ADDR, firmware_size);

    return 1;
}

bool FindFreeAddress(uint32_t size, uint32_t* address) {
    if(!address || size == 0) return false;

    uint32_t current_addr = FIRMWARE_START_ADDRESS;
    uint32_t required_size = size + FIRMWARE_HEADER_SIZE;

    current_addr = (current_addr + AT25_SECTOR_SIZE - 1u) & ~(AT25_SECTOR_SIZE - 1u);

    while(current_addr + required_size <= FIRMWARE_DATA_END_ADDRESS) {
        bool is_free = true;

        for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
            if(firmware_manager.firmware_table[i].valid) {
                uint32_t fw_start = firmware_manager.firmware_table[i].address;
                uint32_t fw_end = fw_start + firmware_manager.firmware_table[i].header.size + FIRMWARE_HEADER_SIZE;
                fw_end = (fw_end + AT25_SECTOR_SIZE - 1u) & ~(AT25_SECTOR_SIZE - 1u);

                if(!(current_addr + required_size <= fw_start || current_addr >= fw_end)) {
                    is_free = false;
                    current_addr = fw_end;
                    break;
                }
            }
        }

        if(is_free) {
            *address = current_addr;
            return true;
        }
    }

    return false;
}

bool Firmware_Init(void) {
    if(!initialized) {
        AT25_Init();
        AT25_Unprotect();

        if(!LoadFirmwareTable()) {
            memset(&firmware_manager, 0, sizeof(FirmwareManager_t));
            firmware_manager.firmware_count = 0;
            SaveFirmwareTable();
        }

        initialized = true;
    }
    return true;
}

bool Firmware_BeginSave(uint32_t address, uint32_t size, uint32_t version, const char* name) {
    if(!initialized) Firmware_Init();
    if(size == 0) return false;
    if(address + size + FIRMWARE_HEADER_SIZE > FIRMWARE_DATA_END_ADDRESS) return false;

    if(!EraseRange4K(address, size + FIRMWARE_HEADER_SIZE)) return false;

    FirmwareHeader_t header;
    memset(&header, 0xFF, sizeof(header));
    header.magic = FIRMWARE_MAGIC;
    header.version = version;
    header.size = size;
    header.crc32 = 0xFFFFFFFFu;       // финальный CRC будет записан в Firmware_FinishSave()
    header.timestamp = 0x00000001u;
    header.status = FIRMWARE_INACTIVE_STATUS;

    memset(header.name, 0, sizeof(header.name));
    if(name) {
        strncpy((char*)header.name, name, sizeof(header.name) - 1u);
    }

    return AT25_WriteBytes(address, (uint8_t*)&header, sizeof(FirmwareHeader_t));
}

bool Firmware_WriteChunk(uint32_t address, uint32_t offset, const uint8_t* data, uint32_t length) {
    if(!initialized) Firmware_Init();
    if(!data || length == 0) return false;
    return AT25_WriteBytes(address + FIRMWARE_HEADER_SIZE + offset, data, length);
}

bool Firmware_FinishSave(uint32_t address, uint32_t crc32) {
    if(!initialized) Firmware_Init();

    FirmwareHeader_t header;
    AT25_ReadData(address, (uint8_t*)&header, sizeof(header));
    if(header.magic != FIRMWARE_MAGIC) return false;

    header.crc32 = crc32;
    if(!AT25_WriteBytes(address, (uint8_t*)&header, sizeof(header))) return false;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid && firmware_manager.firmware_table[i].address == address) {
            firmware_manager.firmware_table[i].header = header;
            firmware_manager.firmware_table[i].valid = 1;
            return SaveFirmwareTable();
        }
    }

    if(firmware_manager.firmware_count >= MAX_FIRMWARE_COUNT) return false;

    firmware_manager.firmware_table[firmware_manager.firmware_count].address = address;
    firmware_manager.firmware_table[firmware_manager.firmware_count].header = header;
    firmware_manager.firmware_table[firmware_manager.firmware_count].valid = 1;
    firmware_manager.firmware_count++;

    return SaveFirmwareTable();
}

bool Firmware_Save(uint32_t address, const uint8_t* data, uint32_t size, uint32_t version, const char* name) {
    if(!data || size == 0) return false;
    uint32_t crc = CalculateCRC32(data, size);
    if(!Firmware_BeginSave(address, size, version, name)) return false;
    if(!Firmware_WriteChunk(address, 0, data, size)) return false;
    return Firmware_FinishSave(address, crc);
}

bool Firmware_Load(uint32_t address, uint8_t* buffer, uint32_t max_size) {
    if(!initialized) Firmware_Init();
    if(!buffer) return false;

    FirmwareHeader_t header;
    AT25_ReadData(address, (uint8_t*)&header, sizeof(header));

    if(header.magic != FIRMWARE_MAGIC) return false;
    if(header.size > max_size) return false;

    AT25_ReadData(address + FIRMWARE_HEADER_SIZE, buffer, header.size);

    uint32_t crc = CalculateCRC32(buffer, header.size);
    return crc == header.crc32;
}

bool Firmware_Activate(uint32_t address) {
    if(!initialized) Firmware_Init();

    bool found = false;
    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid) {
            if(firmware_manager.firmware_table[i].address == address) {
                firmware_manager.firmware_table[i].header.status = FIRMWARE_ACTIVE_STATUS;
                found = true;
            } else {
                firmware_manager.firmware_table[i].header.status = FIRMWARE_INACTIVE_STATUS;
            }
        }
    }

    if(!found) return false;
    return SaveFirmwareTable();
}

bool Firmware_GetActive(FirmwareHeader_t* header) {
    if(!initialized) Firmware_Init();
    if(!header) return false;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid && firmware_manager.firmware_table[i].header.status == FIRMWARE_ACTIVE_STATUS) {
            *header = firmware_manager.firmware_table[i].header;
            return true;
        }
    }
    return false;
}

bool Firmware_Delete(uint32_t address) {
    if(!initialized) Firmware_Init();

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].address == address) {
            uint32_t fw_size = firmware_manager.firmware_table[i].header.size + FIRMWARE_HEADER_SIZE;
            EraseRange4K(address, fw_size);

            for(uint32_t k = i; k < firmware_manager.firmware_count - 1u; k++) {
                firmware_manager.firmware_table[k] = firmware_manager.firmware_table[k + 1u];
            }
            firmware_manager.firmware_count--;
            return SaveFirmwareTable();
        }
    }
    return false;
}

int FlashFirmwareFromAT25_Index(uint32_t index, uint8_t use_oled)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];

    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(index >= count)
        return 0;

    uint32_t firmware_size = headers[index].size;
    uint32_t firmware_addr = addresses[index] + FIRMWARE_HEADER_SIZE;

    if(firmware_size == 0u)
        return 0;

    uint32_t program_size = (firmware_size + 3u) & ~3u;

    if(use_oled)
        OLED_Status("Preparing SWD", "Please wait...");

    if(!swd_prepare_target_for_program(program_size))
        return 0;

    uint8_t buffer[64];
    uint32_t offset = 0;
    uint32_t chunk_id = 0;
    uint32_t total_chunks = (firmware_size + sizeof(buffer) - 1u) / sizeof(buffer);

    while(offset < firmware_size)
    {
        uint32_t chunk = firmware_size - offset;
        if(chunk > sizeof(buffer))
            chunk = sizeof(buffer);

        memset(buffer, 0xFF, sizeof(buffer));
        AT25_ReadData(firmware_addr + offset, buffer, chunk);

        uint32_t chunk_program_size = (chunk + 3u) & ~3u;

        for(uint32_t i = 0; i < chunk_program_size; i += 4u)
        {
            uint32_t word =
                ((uint32_t)buffer[i]) |
                ((uint32_t)buffer[i + 1u] << 8) |
                ((uint32_t)buffer[i + 2u] << 16) |
                ((uint32_t)buffer[i + 3u] << 24);

            if(!flash_program_word32(FLASH_BASE_ADDR + offset + i, word))
            {
                flash_lock_target();
                return 0;
            }
        }

        offset += chunk;
        chunk_id++;

        if(use_oled)
        {
            char line[32];
            snprintf(line, sizeof(line), "%lu/%lu",
                     (unsigned long)chunk_id,
                     (unsigned long)total_chunks);

            OLED_Status("Flashing AT25", line);
        }
        else
        {
            UART_SendByte(RESP_OK);
        }
    }

    flash_lock_target();

    return 1;
}

bool Firmware_Format(void)
{
    if(!initialized) Firmware_Init();

    AT25_ForceUnprotect();

    for(uint32_t addr = FIRMWARE_START_ADDRESS;
        addr < FIRMWARE_DATA_END_ADDRESS;
        addr += AT25_BLOCK_64K_SIZE)
    {
        if(!AT25_Block64KErase(addr))
            return false;
    }

    memset(&firmware_manager, 0, sizeof(FirmwareManager_t));
    firmware_manager.firmware_count = 0;

    return SaveFirmwareTable();
}

bool Firmware_FindByName(const char* name, FirmwareHeader_t* header, uint32_t* address) {
    if(!initialized) Firmware_Init();
    if(!name || !header || !address) return false;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid && strcmp((char*)firmware_manager.firmware_table[i].header.name, name) == 0) {
            *header = firmware_manager.firmware_table[i].header;
            *address = firmware_manager.firmware_table[i].address;
            return true;
        }
    }
    return false;
}

bool Firmware_FindByVersion(uint32_t version, FirmwareHeader_t* header, uint32_t* address) {
    if(!initialized) Firmware_Init();
    if(!header || !address) return false;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid && firmware_manager.firmware_table[i].header.version == version) {
            *header = firmware_manager.firmware_table[i].header;
            *address = firmware_manager.firmware_table[i].address;
            return true;
        }
    }
    return false;
}

uint32_t Firmware_GetList(FirmwareHeader_t* headers, uint32_t* addresses, uint32_t max_count) {
    if(!initialized) Firmware_Init();
    if(!headers || !addresses) return 0;

    uint32_t count = 0;
    for(uint32_t i = 0; i < firmware_manager.firmware_count && count < max_count; i++) {
        if(firmware_manager.firmware_table[i].valid) {
            headers[count] = firmware_manager.firmware_table[i].header;
            addresses[count] = firmware_manager.firmware_table[i].address;
            count++;
        }
    }
    return count;
}

bool Firmware_Verify(uint32_t address, uint32_t* crc32)
{
    if(!initialized) Firmware_Init();

    FirmwareHeader_t header;
    AT25_ReadData(address, (uint8_t*)&header, sizeof(FirmwareHeader_t));

    if(header.magic != FIRMWARE_MAGIC)
        return false;

    uint32_t crc = 0xFFFFFFFF;
    uint8_t buffer[64];

    uint32_t remaining = header.size;
    uint32_t offset = 0;

    while(remaining > 0)
    {
        uint32_t chunk = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;

        AT25_ReadData(address + FIRMWARE_HEADER_SIZE + offset, buffer, chunk);

        for(uint32_t i = 0; i < chunk; i++)
        {
            crc ^= buffer[i];

            for(uint8_t j = 0; j < 8; j++)
            {
                if(crc & 1)
                    crc = (crc >> 1) ^ 0xEDB88320;
                else
                    crc >>= 1;
            }
        }

        offset += chunk;
        remaining -= chunk;
    }

    crc = ~crc;
    *crc32 = crc;

    return crc == header.crc32;
}
bool Firmware_FlashToInternal(uint32_t ext_address, uint32_t int_address, uint32_t size) {
    if(!initialized) Firmware_Init();

    uint8_t* buffer = (uint8_t*)malloc(AT25_PAGE_SIZE);
    if(!buffer) return false;

    uint32_t bytes_read = 0;
    bool success = true;

    while(bytes_read < size) {
        uint32_t chunk_size = (size - bytes_read) > AT25_PAGE_SIZE ? AT25_PAGE_SIZE : (size - bytes_read);
        AT25_ReadData(ext_address + FIRMWARE_HEADER_SIZE + bytes_read, buffer, chunk_size);
        // TODO: Internal_Flash_Write(int_address + bytes_read, buffer, chunk_size);
        bytes_read += chunk_size;
    }

    free(buffer);
    return success;
}

uint32_t Firmware_GetFreeSpace(void) {
    if(!initialized) Firmware_Init();

    uint32_t used_space = 0;
    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid) {
            uint32_t fw_size = firmware_manager.firmware_table[i].header.size + FIRMWARE_HEADER_SIZE;
            used_space += (fw_size + AT25_SECTOR_SIZE - 1u) & ~(AT25_SECTOR_SIZE - 1u);
        }
    }

    return (FIRMWARE_DATA_END_ADDRESS - FIRMWARE_START_ADDRESS) - used_space;
}

void Firmware_GetMemoryInfo(uint32_t* total, uint32_t* used, uint32_t* free) {
    if(!initialized) Firmware_Init();
    if(!total || !used || !free) return;

    *total = FIRMWARE_DATA_END_ADDRESS - FIRMWARE_START_ADDRESS;
    *used = 0;

    for(uint32_t i = 0; i < firmware_manager.firmware_count; i++) {
        if(firmware_manager.firmware_table[i].valid) {
            uint32_t fw_size = firmware_manager.firmware_table[i].header.size + FIRMWARE_HEADER_SIZE;
            *used += (fw_size + AT25_SECTOR_SIZE - 1u) & ~(AT25_SECTOR_SIZE - 1u);
        }
    }

    *free = *total - *used;
}
