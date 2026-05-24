#ifndef UART_FIRMWARE_HANDLER_H
#define UART_FIRMWARE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

// Команды UART
#define CMD_CHECK_TARGET     'C'  // Проверка связи
#define CMD_START_PROGRAM    'S'  // Начать передачу прошивки
#define CMD_END_PROGRAM      'E'  // Завершить передачу прошивки
#define CMD_EEPROM_SAVE      'W'  // Сохранить в EEPROM
#define CMD_EEPROM_COUNT     'N'  // Получить количество прошивок
#define CMD_EEPROM_LIST      'L'  // Получить список прошивок
#define CMD_EEPROM_READ      'R'  // Прочитать прошивку
#define CMD_EEPROM_DELETE    'D'  // Удалить прошивку
#define CMD_EEPROM_ACTIVATE  'A'  // Активировать прошивку
#define CMD_EEPROM_INFO      'I'  // Информация о памяти
#define CMD_EEPROM_VERIFY 'V'
#define RESP_OK              'K'  // Успешно
#define RESP_ERR             'E'  // Ошибка
#define RESP_BUSY            'B'  // Занято
#define RESP_WAIT            'W'  // Ожидание данных
#define CMD_EEPROM_FORMAT    'X'  // Полное форматирование AT25

// Протокол передачи
typedef struct {
    uint32_t firmware_size;    // Размер прошивки
    uint32_t firmware_version; // Версия прошивки
    char     firmware_name[32]; // Имя прошивки
    uint32_t crc32;            // CRC32 для проверки
} FirmwareTransferHeader_t;

void USART2_Init(void);
// Инициализация обработчика
void UART_Firmware_Init(void);

// Обработка команд
void UART_Firmware_ProcessCommand(uint8_t cmd);

// Отправка данных через UART
void UART_SendByte(uint8_t data);
void UART_SendData(const uint8_t* data, uint32_t length);
void UART_SendString(const char* str);
void UART_SendResponse(uint8_t response);
void UART_SendNumber(uint32_t number);

// Получение данных
uint8_t UART_ReceiveByte(void);
bool UART_ReceiveData(uint8_t* buffer, uint32_t length, uint32_t timeout_ms);

// Обработчики команд
void CMD_CheckTarget(void);
void CMD_StartProgram(void);
void CMD_EndProgram(void);
void CMD_SaveToEEPROM(void);
void CMD_GetFirmwareCount(void);
void CMD_ListFirmwares(void);
void CMD_ReadFirmware(void);
void CMD_DeleteFirmware(void);
void CMD_ActivateFirmware(void);
void CMD_GetMemoryInfo(void);
void CMD_VerifyFirmware(void);
void CMD_FormatMemory(void);

void UART_SendU32(uint32_t value);

int UART_ReceiveExactLocal(uint8_t *buffer, uint32_t length, uint32_t timeout_loops);

void CMD_DirectSwdCheckTarget(void);

void CMD_DirectSwdProgramFile(void);

void CMD_DirectSwdEndProgram(void);

void CMD_FlashFirmwareFromAT25(void);

void UART_SendByte(uint8_t data);

void UART_SendString(const char* str);

uint8_t UART_ReceiveByte(void);

#endif // UART_FIRMWARE_HANDLER_H