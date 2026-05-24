#ifndef AT25DF321A_FIRMWARE_H
#define AT25DF321A_FIRMWARE_H

#include <stdint.h>
#include <stdbool.h>
#include "oled_driver.h"
#include "oled_ui.h"
#include <stdio.h>
#include "uart_firmware_handler.h"
// Адресное пространство для хранения прошивок
#define FIRMWARE_START_ADDRESS     0x00000000
#define FIRMWARE_HEADER_SIZE       64           // Размер заголовка прошивки
#define MAX_FIRMWARE_COUNT         16           // Максимальное количество прошивок

// Вся доступная память (4MB)
#define TOTAL_MEMORY_SIZE          (4 * 1024 * 1024)  // 4 MB
#define FIRMWARE_AREA_END          (TOTAL_MEMORY_SIZE - 1)

// Структура заголовка прошивки
typedef struct {
    uint32_t magic;           // Магическое число (0xDEADBEEF)
    uint32_t version;         // Версия прошивки
    uint32_t size;            // Размер прошивки в байтах (без ограничений)
    uint32_t crc32;           // CRC32 прошивки
    uint32_t timestamp;       // Время создания
    uint8_t  name[32];        // Имя прошивки
    uint8_t  status;          // Статус (0 - неактивна, 1 - активна)
    uint8_t  reserved[15];    // Зарезервировано
} FirmwareHeader_t;

// Структура для управления прошивками
typedef struct {
    uint32_t firmware_count;
    struct {
        uint32_t address;
        FirmwareHeader_t header;
        uint8_t valid;
    } firmware_table[MAX_FIRMWARE_COUNT];
} FirmwareManager_t;

// Инициализация менеджера прошивок
bool Firmware_Init(void);

// Поиск свободного адреса для прошивки
bool FindFreeAddress(uint32_t size, uint32_t* address);

// Сохранение прошивки (без ограничения по размеру)
bool Firmware_Save(uint32_t address, const uint8_t* data, uint32_t size, 
                   uint32_t version, const char* name);

// Загрузка прошивки
bool Firmware_Load(uint32_t address, uint8_t* buffer, uint32_t max_size);

// Активация прошивки
bool Firmware_Activate(uint32_t address);

// Получение активной прошивки
bool Firmware_GetActive(FirmwareHeader_t* header);

// Удаление прошивки
bool Firmware_Delete(uint32_t address);

// Форматирование области прошивок
bool Firmware_Format(void);

// Поиск прошивки по имени
bool Firmware_FindByName(const char* name, FirmwareHeader_t* header, uint32_t* address);

// Поиск прошивки по версии
bool Firmware_FindByVersion(uint32_t version, FirmwareHeader_t* header, uint32_t* address);

// Получение списка всех прошивок
uint32_t Firmware_GetList(FirmwareHeader_t* headers, uint32_t* addresses, uint32_t max_count);

// Верификация прошивки
bool Firmware_Verify(uint32_t address, uint32_t* crc32);

// Копирование прошивки из внешней памяти во внутреннюю
bool Firmware_FlashToInternal(uint32_t ext_address, uint32_t int_address, uint32_t size);

// Получение свободного места
uint32_t Firmware_GetFreeSpace(void);

// Получение информации о памяти
void Firmware_GetMemoryInfo(uint32_t* total, uint32_t* used, uint32_t* free);

int FlashFirmwareFromAT25_Index(uint32_t index, uint8_t use_oled);

int swd_prepare_target_for_program(uint32_t firmware_size);

#endif // AT25DF321A_FIRMWARE_H