#ifndef AT25DF321A_H
#define AT25DF321A_H

#include <stdint.h>
#include <stdbool.h>

// Команды AT25DF321A
#define AT25_CMD_READ_ID         0x9F
#define AT25_CMD_READ_STATUS     0x05
#define AT25_CMD_WRITE_STATUS    0x01
#define AT25_CMD_READ_DATA       0x03
#define AT25_CMD_FAST_READ       0x0B
#define AT25_CMD_PAGE_PROGRAM    0x02
#define AT25_CMD_BLOCK_ERASE_4K  0x20
#define AT25_CMD_BLOCK_ERASE_32K 0x52
#define AT25_CMD_BLOCK_ERASE_64K 0xD8
#define AT25_CMD_CHIP_ERASE      0xC7
#define AT25_CMD_WRITE_ENABLE    0x06
#define AT25_CMD_WRITE_DISABLE   0x04
#define AT25_CMD_DEEP_POWER_DOWN 0xB9
#define AT25_CMD_RESUME          0xAB

// Статус биты
#define AT25_STATUS_BUSY         (1 << 0)
#define AT25_STATUS_WRITE_ENABLE (1 << 1)
#define AT25_STATUS_BP0          (1 << 2)
#define AT25_STATUS_BP1          (1 << 3)
#define AT25_STATUS_BP2          (1 << 4)
#define AT25_STATUS_BP3          (1 << 5)
#define AT25_STATUS_WRITE_PROTECT (1 << 7)

// Размеры памяти
#define AT25_PAGE_SIZE           256
#define AT25_PAGES_PER_SECTOR    16   // 4KB sector = 16 pages
#define AT25_SECTOR_SIZE         (AT25_PAGE_SIZE * AT25_PAGES_PER_SECTOR) // 4096 bytes
#define AT25_BLOCK_32K_SIZE      (32 * 1024)
#define AT25_BLOCK_64K_SIZE      (64 * 1024)
#define AT25_TOTAL_SIZE          (32 * 1024 * 1024 / 8) // 4MB (32 Mbit)

// Инициализация SPI для AT25DF321A
void AT25_Init(void);

// Базовые функции
uint8_t AT25_ReadStatus(void);
bool AT25_WaitForReady(uint32_t timeout_ms);
void AT25_WriteEnable(void);
void AT25_WriteDisable(void);

// Чтение данных
void AT25_ReadData(uint32_t address, uint8_t* buffer, uint32_t length);
uint8_t AT25_ReadByte(uint32_t address);

// Запись данных
bool AT25_PageProgram(uint32_t address, const uint8_t* data, uint16_t length);
bool AT25_WriteBytes(uint32_t address, const uint8_t* data, uint32_t length);

// Стирание
bool AT25_SectorErase(uint32_t address);
bool AT25_Block32KErase(uint32_t address);
bool AT25_Block64KErase(uint32_t address);
bool AT25_ChipErase(void);

// Идентификация
void AT25_ReadID(uint8_t* manufacturer, uint8_t* device1, uint8_t* device2);

// Тестовые функции с выводом через USART
void AT25_TestAll(void);
void AT25_TestReadID(void);
void AT25_TestWriteRead(uint32_t start_address, const uint8_t* test_data, uint16_t length);
void AT25_TestErase(uint32_t address);
void AT25_Unprotect(void);
void AT25_ForceUnprotect(void);
#endif // AT25DF321A_H