#include "../inc/uart_firmware_handler.h"
#include "../inc/at25df321a.h"
#include "../inc/at25df321a_firmware.h"
#include "../inc/stm32f1xx.h"
#include <string.h>
#include <stdlib.h>

extern void UART_SendByte(uint8_t data);
extern void UART_SendString(const char* str);
extern uint8_t UART_ReceiveByte(void);

// Эти 3 функции добавлены в исправленный at25df321a_firmware.c.
extern bool Firmware_BeginSave(uint32_t address, uint32_t size, uint32_t version, const char* name);
extern bool Firmware_WriteChunk(uint32_t address, uint32_t offset, const uint8_t* data, uint32_t length);
extern bool Firmware_FinishSave(uint32_t address, uint32_t crc32);

#define UART_RX_POLL_DELAY       100u
#define DEFAULT_FIRMWARE_VERSION 1u
#define EEPROM_CHUNK_SIZE        64u

static uint32_t crc32_table[256];
static bool crc32_initialized = false;

static void InitCRC32Table(void) {
    if(crc32_initialized) return;

    for(uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for(int j = 0; j < 8; j++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
        crc32_table[i] = crc;
    }
    crc32_initialized = true;
}

static uint32_t CRC32_InitValue(void) {
    InitCRC32Table();
    return 0xFFFFFFFFu;
}

static uint32_t CRC32_Update(uint32_t crc, const uint8_t* data, uint32_t length) {
    for(uint32_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFFu];
    }
    return crc;
}

static uint32_t CRC32_Final(uint32_t crc) {
    return ~crc;
}

void UART_SendData(const uint8_t* data, uint32_t length) {
    for(uint32_t i = 0; i < length; i++) UART_SendByte(data[i]);
}

void UART_SendResponse(uint8_t response) {
    UART_SendByte(response);
}

void UART_SendNumber(uint32_t number) {
    char buffer[16];
    int len = 0;

    if(number == 0) {
        UART_SendByte('0');
        return;
    }

    while(number > 0 && len < (int)sizeof(buffer)) {
        buffer[len++] = (char)('0' + (number % 10u));
        number /= 10u;
    }

    for(int i = len - 1; i >= 0; i--) UART_SendByte((uint8_t)buffer[i]);
}

bool UART_ReceiveData(uint8_t* buffer, uint32_t length, uint32_t timeout_ms) {
    if(!buffer) return false;

    uint32_t received = 0;
    uint32_t timeout = timeout_ms * 1000u;

    while(received < length) {
        if(USART2->SR & USART_SR_RXNE) {
            buffer[received++] = (uint8_t)USART2->DR;
            timeout = timeout_ms * 1000u;
        } else {
            if(timeout-- == 0) return false;
            for(volatile uint32_t i = 0; i < UART_RX_POLL_DELAY; i++);
        }
    }

    return true;
}

void UART_Firmware_Init(void) {
    Firmware_Init();
   
}

void CMD_FormatMemory(void)
{
    UART_SendResponse(RESP_OK);
    bool ok = Firmware_Format();
    UART_SendResponse(ok ? RESP_OK : RESP_ERR);
}

void UART_Firmware_ProcessCommand(uint8_t cmd) {
    switch(cmd) {
        case CMD_CHECK_TARGET:    CMD_CheckTarget(); break;
        case CMD_START_PROGRAM:   CMD_StartProgram(); break;
        case CMD_END_PROGRAM:     CMD_EndProgram(); break;
        case CMD_EEPROM_SAVE:     CMD_SaveToEEPROM(); break;
        case CMD_EEPROM_COUNT:    CMD_GetFirmwareCount(); break;
        case CMD_EEPROM_LIST:     CMD_ListFirmwares(); break;
        case CMD_EEPROM_READ:     CMD_ReadFirmware(); break;
        case CMD_EEPROM_DELETE:   CMD_DeleteFirmware(); break;
        case CMD_EEPROM_ACTIVATE: CMD_ActivateFirmware(); break;
        case CMD_EEPROM_INFO:     CMD_GetMemoryInfo(); break;
        case CMD_EEPROM_VERIFY:   CMD_VerifyFirmware();break;
        case CMD_EEPROM_FORMAT:    CMD_FormatMemory(); break;
        default:                  UART_SendResponse(RESP_ERR); break;
    }
}

void CMD_VerifyFirmware(void)
{
    uint32_t index;

    if(!UART_ReceiveData((uint8_t*)&index, 4, 1000)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(index >= count) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    UART_SendResponse(RESP_OK);

    uint32_t crc = 0;
    bool ok = Firmware_Verify(addresses[index], &crc);

    if(!ok) {
        // CRC всё равно отправляем, чтобы ПО не висело
    }

    UART_SendData((uint8_t*)&crc, 4);
}

void CMD_CheckTarget(void) {
    UART_SendResponse(RESP_OK);
}

// Старый режим прямой прошивки не был реализован на STM32.
// Оставлен как корректный hand-shake, чтобы PC не зависал.
void CMD_StartProgram(void) {
    uint32_t total_chunks;
    if(!UART_ReceiveData((uint8_t*)&total_chunks, 4, 1000)) {
        UART_SendResponse(RESP_ERR);
        return;
    }
    (void)total_chunks;
    UART_SendResponse(RESP_OK);
}

void CMD_EndProgram(void) {
    UART_SendResponse(RESP_OK);
}

void CMD_SaveToEEPROM(void) {
    FirmwareTransferHeader_t transfer;
    uint32_t save_address;
    uint32_t received = 0;
    uint32_t crc;
    uint8_t buffer[EEPROM_CHUNK_SIZE];

    // C# клиент после 'W' ждёт готовность.
    UART_SendResponse(RESP_OK);

    // Теперь принимаем не только размер, а весь заголовок:
    // size + version + name[32] + crc32
    if(!UART_ReceiveData((uint8_t*)&transfer,
                         sizeof(FirmwareTransferHeader_t),
                         3000) ||
       transfer.firmware_size == 0) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    transfer.firmware_name[31] = '\0';

    if(transfer.firmware_version == 0) {
        transfer.firmware_version = DEFAULT_FIRMWARE_VERSION;
    }

    if(!FindFreeAddress(transfer.firmware_size, &save_address)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    if(!Firmware_BeginSave(save_address,
                           transfer.firmware_size,
                           transfer.firmware_version,
                           transfer.firmware_name)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    // заголовок принят, память найдена и стёрта
    UART_SendResponse(RESP_OK);

    crc = CRC32_InitValue();

    while(received < transfer.firmware_size) {
        uint32_t chunk_size =
            (transfer.firmware_size - received) > EEPROM_CHUNK_SIZE
            ? EEPROM_CHUNK_SIZE
            : (transfer.firmware_size - received);

        if(!UART_ReceiveData(buffer, chunk_size, 5000)) {
            UART_SendResponse(RESP_ERR);
            return;
        }

        crc = CRC32_Update(crc, buffer, chunk_size);

        if(!Firmware_WriteChunk(save_address, received, buffer, chunk_size)) {
            UART_SendResponse(RESP_ERR);
            return;
        }

        received += chunk_size;
        UART_SendResponse(RESP_OK);
    }

    uint32_t final_crc = CRC32_Final(crc);

    // Можно сравнить с CRC от ПК, если он его передаёт
    if(transfer.crc32 != 0 && transfer.crc32 != final_crc) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    if(Firmware_FinishSave(save_address, final_crc)) {
        UART_SendResponse(RESP_OK);
    } else {
        UART_SendResponse(RESP_ERR);
    }
}

void CMD_GetFirmwareCount(void) {
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);
    UART_SendData((uint8_t*)&count, 4);
}

void CMD_ListFirmwares(void) {
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    for(uint32_t i = 0; i < count; i++) {
        UART_SendString("Firmware #");
        UART_SendNumber(i + 1u);
        UART_SendString(": ");
        UART_SendString((char*)headers[i].name);
        UART_SendString(" v");
        UART_SendNumber(headers[i].version);
        UART_SendString(" (");
        UART_SendNumber(headers[i].size);
        UART_SendString(" bytes)");
        if(headers[i].status == 1u) UART_SendString(" ACTIVE");
        UART_SendString("\r\n");
    }
}

void CMD_ReadFirmware(void) {
    uint32_t index;
    if(!UART_ReceiveData((uint8_t*)&index, 4, 1000)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(index >= count) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    UART_SendData((uint8_t*)&headers[index].size, 4);

    uint8_t buffer[256];
    for(uint32_t i = 0; i < headers[index].size; i += sizeof(buffer)) {
        uint32_t chunk = (headers[index].size - i) > sizeof(buffer) ? sizeof(buffer) : (headers[index].size - i);
        AT25_ReadData(addresses[index] + FIRMWARE_HEADER_SIZE + i, buffer, chunk);
        UART_SendData(buffer, chunk);
    }
}

void CMD_DeleteFirmware(void) {
    uint32_t index;
    if(!UART_ReceiveData((uint8_t*)&index, 4, 1000)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(index >= count) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    UART_SendResponse(Firmware_Delete(addresses[index]) ? RESP_OK : RESP_ERR);
}

void CMD_ActivateFirmware(void) {
    uint32_t index;
    if(!UART_ReceiveData((uint8_t*)&index, 4, 1000)) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(index >= count) {
        UART_SendResponse(RESP_ERR);
        return;
    }

    UART_SendResponse(Firmware_Activate(addresses[index]) ? RESP_OK : RESP_ERR);
}

void CMD_GetMemoryInfo(void) {
    uint32_t total, used, free_space;
    Firmware_GetMemoryInfo(&total, &used, &free_space);

    UART_SendString("Total: "); UART_SendNumber(total); UART_SendString(" bytes\r\n");
    UART_SendString("Used: ");  UART_SendNumber(used);  UART_SendString(" bytes\r\n");
    UART_SendString("Free: ");  UART_SendNumber(free_space); UART_SendString(" bytes\r\n");
}
