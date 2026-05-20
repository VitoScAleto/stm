#include "../inc/swd.h"
#include "at25df321a.h"

static uint8_t spi_initialized = 0;

// Прототипы статических функций
static void SPI_Delay(void);
static void SPI_CS_Low(void);
static void SPI_CS_High(void);
static uint8_t SPI_TransferByte(uint8_t data);
static void SPI_DiagnosticPins(void);
static void SPI1_Init(void);

// Вспомогательные функции
static void SPI_Delay(void) {
    for(volatile int i = 0; i < 100; i++);
}

static void SPI_CS_Low(void) {
    GPIOA->BRR = GPIO_BRR_BR4;
    SPI_Delay();
}

static void SPI_CS_High(void) {
    SPI_Delay();
    GPIOA->BSRR = GPIO_BSRR_BS4;
    SPI_Delay();
}

static uint8_t SPI_TransferByte(uint8_t data) {
    volatile uint32_t timeout = 100000;
    
    // Очищаем флаги ошибок
    SPI1->SR = 0;
    
    // Ждем пока TX буфер пуст
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if(--timeout == 0) {
            TxStr2((uint8_t*)"ERROR: SPI TX timeout!\r\n");
            return 0xFF;
        }
    }
    
    // Отправляем данные
    SPI1->DR = data;
    
    timeout = 100000;
    // Ждем завершения приема
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if(--timeout == 0) {
            TxStr2((uint8_t*)"ERROR: SPI RX timeout!\r\n");
            return 0xFF;
        }
    }
    
    // Проверяем ошибки
    if(SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF)) {
        TxStr2((uint8_t*)"ERROR: SPI overrun or mode fault!\r\n");
    }
    
    return SPI1->DR;
}

static void SPI_DiagnosticPins(void) {
    TxStr2((uint8_t*)"\r\n=== SPI Pin Diagnostic ===\r\n");
    
    uint32_t crl = GPIOA->CRL;
    uint32_t odr = GPIOA->ODR;
    
    TxLabelHex32((char*)"GPIOA->CRL: ", crl);
    TxLabelHex32((char*)"GPIOA->ODR: ", odr);
    
    TxStr2((uint8_t*)"PA4 (CS): ");
    if(crl & GPIO_CRL_MODE4) {
        TxStr2((uint8_t*)"Output, ");
        if(crl & GPIO_CRL_CNF4_0) TxStr2((uint8_t*)"Push-Pull\r\n");
        else TxStr2((uint8_t*)"Open-Drain\r\n");
    } else {
        TxStr2((uint8_t*)"Input\r\n");
    }
    
    TxStr2((uint8_t*)"PA5 (SCK): ");
    if(crl & GPIO_CRL_MODE5) {
        TxStr2((uint8_t*)"Output, ");
        if(crl & GPIO_CRL_CNF5_0) TxStr2((uint8_t*)"Push-Pull\r\n");
        else TxStr2((uint8_t*)"AF Push-Pull\r\n");
    } else {
        TxStr2((uint8_t*)"Input\r\n");
    }
    
    TxStr2((uint8_t*)"CS pin state: ");
    if(GPIOA->ODR & GPIO_ODR_ODR4) TxStr2((uint8_t*)"HIGH\r\n");
    else TxStr2((uint8_t*)"LOW\r\n");
}

static void SPI1_Init(void) {
    TxStr2((uint8_t*)"\r\nInitializing SPI1...\r\n");
    
    // Включаем тактирование
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
    
    // Небольшая задержка
    for(volatile int i = 0; i < 1000; i++);
    
    // Сброс SPI1
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    for(volatile int i = 0; i < 100; i++);
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    
    // Настройка пинов SPI
    // PA4 - CS (General Purpose Output)
    GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1; // Output 50MHz
    GPIOA->CRL &= ~GPIO_CRL_CNF4_0;  // Push-Pull
    GPIOA->CRL &= ~GPIO_CRL_CNF4_1;
    GPIOA->BSRR = GPIO_BSRR_BS4;     // CS HIGH
    
    // PA5 - SCK (Alternate Function Push-Pull)
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1; // Output 50MHz
    GPIOA->CRL |= GPIO_CRL_CNF5_1;   // AF Push-Pull
    GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
    
    // PA6 - MISO (Input Floating)
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_CNF6_0;   // Input floating
    GPIOA->CRL &= ~GPIO_CRL_MODE6;
    
    // PA7 - MOSI (Alternate Function Push-Pull)
    GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1; // Output 50MHz
    GPIOA->CRL |= GPIO_CRL_CNF7_1;   // AF Push-Pull
    GPIOA->CRL &= ~GPIO_CRL_CNF7_0;
    
    // Настройка SPI1
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    
    // Master mode, Software slave management
    SPI1->CR1 |= SPI_CR1_MSTR;      // Master mode
    SPI1->CR1 |= SPI_CR1_SSI;       // Internal slave select
    SPI1->CR1 |= SPI_CR1_SSM;       // Software slave management
    
    // Baud rate: Fpclk/8 = 36MHz/8 = 4.5MHz (для стабильности)
    SPI1->CR1 |= SPI_CR1_BR_0;      // BR[2:0] = 001
    SPI1->CR1 &= ~SPI_CR1_BR_1;
    SPI1->CR1 &= ~SPI_CR1_BR_2;
    
    // Mode 0: CPOL=0, CPHA=0
    SPI1->CR1 &= ~SPI_CR1_CPOL;
    SPI1->CR1 &= ~SPI_CR1_CPHA;
    
    // 8-bit data frame
    SPI1->CR1 &= ~SPI_CR1_DFF;
    
    // MSB first
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
    
    // Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
    
    // Проверяем что SPI включился
    if(SPI1->CR1 & SPI_CR1_SPE) {
        TxStr2((uint8_t*)"✓ SPI1 initialized successfully\r\n");
        spi_initialized = 1;
    } else {
        TxStr2((uint8_t*)"✗ SPI1 initialization failed!\r\n");
    }
    
    // Диагностика пинов
    SPI_DiagnosticPins();
    
    // Тест SPI - отправка и прием
    TxStr2((uint8_t*)"Testing SPI communication...\r\n");
    uint8_t test_byte = 0xA5;
    uint8_t response;
    
    SPI_CS_Low();
    response = SPI_TransferByte(test_byte);
    SPI_CS_High();
    
    TxLabelHex32((char*)"Test sent: ", test_byte);
    TxLabelHex32((char*)"Test received: ", response);
}

// Публичные функции
void AT25_Init(void) {
    SPI1_Init();
    
    if(!spi_initialized) {
        TxStr2((uint8_t*)"ERROR: SPI not initialized!\r\n");
        return;
    }
    
    TxStr2((uint8_t*)"\r\nInitializing AT25DF321A...\r\n");
    
    // Ждем выхода чипа из возможного режима сна
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_RESUME);  // Команда выхода из deep power-down
    SPI_CS_High();
    
    for(volatile int i = 0; i < 10000; i++);
    
    // Читаем статус несколько раз для проверки связи
    for(int i = 0; i < 3; i++) {
        uint8_t status = AT25_ReadStatus();
        TxLabelHex32((char*)"Status Register read ", status);
        TxStr2((uint8_t*)" (attempt ");
        // Временно используем TxHex32 вместо TxDec32
        TxHex32(i+1);
        TxStr2((uint8_t*)")\r\n");
    }
    
    TxStr2((uint8_t*)"AT25DF321A Initialization complete\r\n");
}

uint8_t AT25_ReadStatus(void) {
    if(!spi_initialized) {
        TxStr2((uint8_t*)"ERROR: SPI not initialized!\r\n");
        return 0xFF;
    }
    
    uint8_t status;
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_READ_STATUS);
    status = SPI_TransferByte(0xFF);
    SPI_CS_High();
    
    return status;
}

bool AT25_WaitForReady(uint32_t timeout_ms) {
    uint32_t timeout = timeout_ms * 1000;
    uint8_t status;
    
    while(timeout--) {
        status = AT25_ReadStatus();
        if(!(status & AT25_STATUS_BUSY)) {
            return true;
        }
        // Небольшая задержка
        for(volatile int i = 0; i < 100; i++);
    }
    
    TxStr2((uint8_t*)"ERROR: Timeout waiting for ready! Status: ");
    TxHex8(status);
    TxStr2((uint8_t*)"\r\n");
    return false;
}

void AT25_WriteEnable(void) {
    if(!spi_initialized) return;
    
    TxStr2((uint8_t*)"Enabling write...\r\n");
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_WRITE_ENABLE);
    SPI_CS_High();
    
    // Небольшая задержка
    for(volatile int i = 0; i < 1000; i++);
    
    // Проверяем что write enable установился
    uint8_t status = AT25_ReadStatus();
    if(status & AT25_STATUS_WRITE_ENABLE) {
        TxStr2((uint8_t*)"✓ Write enable bit set\r\n");
    } else {
        TxStr2((uint8_t*)"✗ Write enable bit NOT set! Status: ");
        TxHex8(status);
        TxStr2((uint8_t*)"\r\n");
    }
}

void AT25_WriteDisable(void) {
    if(!spi_initialized) return;
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_WRITE_DISABLE);
    SPI_CS_High();
}

void AT25_ReadData(uint32_t address, uint8_t* buffer, uint32_t length) {
    if(!spi_initialized) return;
    if(!AT25_WaitForReady(100)) return;
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_READ_DATA);
    SPI_TransferByte((address >> 16) & 0xFF);
    SPI_TransferByte((address >> 8) & 0xFF);
    SPI_TransferByte(address & 0xFF);
    
    for(uint32_t i = 0; i < length; i++) {
        buffer[i] = SPI_TransferByte(0xFF);
    }
    
    SPI_CS_High();
}

uint8_t AT25_ReadByte(uint32_t address) {
    uint8_t data;
    AT25_ReadData(address, &data, 1);
    return data;
}

bool AT25_PageProgram(uint32_t address, const uint8_t* data, uint16_t length) {
    if(!spi_initialized) return false;
    
    TxStr2((uint8_t*)"Page program at address: ");
    TxHex32(address);
    TxStr2((uint8_t*)", length: ");
    TxHex16(length);
    TxStr2((uint8_t*)"\r\n");
    
    if(length > AT25_PAGE_SIZE || (address + length) > ((address & ~(AT25_PAGE_SIZE - 1)) + AT25_PAGE_SIZE)) {
        TxStr2((uint8_t*)"ERROR: Page program crosses page boundary\r\n");
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    
    // Показываем статус до записи
    uint8_t status_before = AT25_ReadStatus();
    TxLabelHex32((char*)"Status before write: ", status_before);
    
    AT25_WriteEnable();
    
    // Проверяем WEL бит
    uint8_t status_after_we = AT25_ReadStatus();
    if(!(status_after_we & AT25_STATUS_WRITE_ENABLE)) {
        TxStr2((uint8_t*)"ERROR: Write enable failed! Status: ");
        TxHex8(status_after_we);
        TxStr2((uint8_t*)"\r\n");
        return false;
    }
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_PAGE_PROGRAM);
    SPI_TransferByte((address >> 16) & 0xFF);
    SPI_TransferByte((address >> 8) & 0xFF);
    SPI_TransferByte(address & 0xFF);
    
    for(uint16_t i = 0; i < length; i++) {
        SPI_TransferByte(data[i]);
    }
    
    SPI_CS_High();
    
    bool result = AT25_WaitForReady(100);
    
    // Показываем статус после записи
    uint8_t status_after = AT25_ReadStatus();
    TxLabelHex32((char*)"Status after write: ", status_after);
    
    return result;
}

bool AT25_WriteBytes(uint32_t address, const uint8_t* data, uint32_t length) {
    if(!spi_initialized) return false;
    
    uint32_t bytes_written = 0;
    uint16_t page_offset = address % AT25_PAGE_SIZE;
    uint16_t bytes_to_write;
    
    while(bytes_written < length) {
        if(page_offset + (length - bytes_written) > AT25_PAGE_SIZE) {
            bytes_to_write = AT25_PAGE_SIZE - page_offset;
        } else {
            bytes_to_write = length - bytes_written;
        }
        
        if(!AT25_PageProgram(address + bytes_written, data + bytes_written, bytes_to_write)) {
            TxStr2((uint8_t*)"ERROR: Page program failed at offset ");
            TxHex32(bytes_written);
            TxStr2((uint8_t*)"\r\n");
            return false;
        }
        
        bytes_written += bytes_to_write;
        page_offset = 0;
    }
    
    return true;
}

bool AT25_SectorErase(uint32_t address) {
    if(!spi_initialized) return false;
    
    if(address & (AT25_SECTOR_SIZE - 1)) {
        TxStr2((uint8_t*)"ERROR: Address not aligned to sector (4KB)\r\n");
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    AT25_WriteEnable();
    
    if(!(AT25_ReadStatus() & AT25_STATUS_WRITE_ENABLE)) {
        TxStr2((uint8_t*)"ERROR: Write enable failed for sector erase\r\n");
        return false;
    }
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_BLOCK_ERASE_4K);
    SPI_TransferByte((address >> 16) & 0xFF);
    SPI_TransferByte((address >> 8) & 0xFF);
    SPI_TransferByte(address & 0xFF);
    SPI_CS_High();
    
    return AT25_WaitForReady(3000);
}

bool AT25_Block32KErase(uint32_t address) {
    if(!spi_initialized) return false;
    
    if(address & (AT25_BLOCK_32K_SIZE - 1)) {
        TxStr2((uint8_t*)"ERROR: Address not aligned to 32KB block\r\n");
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    AT25_WriteEnable();
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_BLOCK_ERASE_32K);
    SPI_TransferByte((address >> 16) & 0xFF);
    SPI_TransferByte((address >> 8) & 0xFF);
    SPI_TransferByte(address & 0xFF);
    SPI_CS_High();
    
    return AT25_WaitForReady(5000);
}

bool AT25_Block64KErase(uint32_t address) {
    if(!spi_initialized) return false;
    
    if(address & (AT25_BLOCK_64K_SIZE - 1)) {
        TxStr2((uint8_t*)"ERROR: Address not aligned to 64KB block\r\n");
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    AT25_WriteEnable();
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_BLOCK_ERASE_64K);
    SPI_TransferByte((address >> 16) & 0xFF);
    SPI_TransferByte((address >> 8) & 0xFF);
    SPI_TransferByte(address & 0xFF);
    SPI_CS_High();
    
    return AT25_WaitForReady(10000);
}

bool AT25_ChipErase(void) {
    if(!spi_initialized) return false;
    
    TxStr2((uint8_t*)"Chip erase started, this will take ~30 seconds...\r\n");
    
    if(!AT25_WaitForReady(100)) return false;
    AT25_WriteEnable();
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_CHIP_ERASE);
    SPI_CS_High();
    
    return AT25_WaitForReady(30000);
}

void AT25_ReadID(uint8_t* manufacturer, uint8_t* device1, uint8_t* device2) {
    if(!spi_initialized) {
        *manufacturer = *device1 = *device2 = 0;
        return;
    }
    
    if(!AT25_WaitForReady(100)) {
        *manufacturer = *device1 = *device2 = 0;
        return;
    }
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_READ_ID);
    *manufacturer = SPI_TransferByte(0xFF);
    *device1 = SPI_TransferByte(0xFF);
    *device2 = SPI_TransferByte(0xFF);
    SPI_CS_High();
}

void AT25_CheckHardware(void) {
    TxStr2((uint8_t*)"\r\n=== Hardware Connection Check ===\r\n");
    
    TxStr2((uint8_t*)"1. Check power and connections:\r\n");
    TxStr2((uint8_t*)"   - VCC to 3.3V\r\n");
    TxStr2((uint8_t*)"   - GND to GND\r\n");
    TxStr2((uint8_t*)"   - CS (PA4) to CS pin\r\n");
    TxStr2((uint8_t*)"   - SCK (PA5) to SCK pin\r\n");
    TxStr2((uint8_t*)"   - MOSI (PA7) to MOSI pin\r\n");
    TxStr2((uint8_t*)"   - MISO (PA6) to MISO pin\r\n");
    TxStr2((uint8_t*)"   - WP and HOLD pins to VCC (3.3V)\r\n");
    
    TxStr2((uint8_t*)"\r\n2. Sending test command...\r\n");
    
    uint8_t test_result = 0;
    SPI_CS_Low();
    SPI_TransferByte(0x00);
    test_result = SPI_TransferByte(0xFF);
    SPI_CS_High();
    
    TxLabelHex32((char*)"Test response (should be 0xFF or something): ", test_result);
    
    TxStr2((uint8_t*)"\r\nIf all IDs are 0x00, check:\r\n");
    TxStr2((uint8_t*)"  - MISO connection (PA6)\r\n");
    TxStr2((uint8_t*)"  - Chip power supply\r\n");
    TxStr2((uint8_t*)"  - WP and HOLD pins are high\r\n");
}

// Тестовые функции
void AT25_TestReadID(void) {
    uint8_t manuf, dev1, dev2;
    
    TxStr2((uint8_t*)"\r\n=== Testing Read ID ===\r\n");
    AT25_ReadID(&manuf, &dev1, &dev2);
    
    TxLabelHex32((char*)"Manufacturer ID: ", manuf);
    TxLabelHex32((char*)"Device ID 1: ", dev1);
    TxLabelHex32((char*)"Device ID 2: ", dev2);
    
    if(manuf == 0x1F && dev1 == 0x47) {
        TxStr2((uint8_t*)"✓ Device ID matches AT25DF321A\r\n");
    } else {
        TxStr2((uint8_t*)"✗ Unexpected Device ID!\r\n");
    }
}

void AT25_TestWriteRead(uint32_t start_address, const uint8_t* test_data, uint16_t length) {
    uint8_t read_buffer[256];
    
    TxStr2((uint8_t*)"\r\n=== Testing Write/Read ===\r\n");
    TxLabelHex32((char*)"Start Address: ", start_address);
    TxStr2((uint8_t*)"Data to write: ");
    for(uint16_t i = 0; i < length; i++) {
        TxHex8(test_data[i]);
        Tx2(' ');
    }
    TxStr2((uint8_t*)"\r\n");
    
    uint32_t sector_addr = start_address & ~(AT25_SECTOR_SIZE - 1);
    TxLabelHex32((char*)"Erasing sector at: ", sector_addr);
    
    if(!AT25_SectorErase(sector_addr)) {
        TxStr2((uint8_t*)"✗ Erase failed!\r\n");
        return;
    }
    TxStr2((uint8_t*)"✓ Sector erased\r\n");
    
    TxStr2((uint8_t*)"Writing data...\r\n");
    if(!AT25_WriteBytes(start_address, test_data, length)) {
        TxStr2((uint8_t*)"✗ Write failed!\r\n");
        return;
    }
    TxStr2((uint8_t*)"✓ Write successful\r\n");
    
    TxStr2((uint8_t*)"Reading data back...\r\n");
    AT25_ReadData(start_address, read_buffer, length);
    
    bool match = true;
    for(uint16_t i = 0; i < length; i++) {
        if(read_buffer[i] != test_data[i]) {
            TxStr2((uint8_t*)"✗ Data mismatch at offset ");
            TxHex16(i);
            TxStr2((uint8_t*)" - Expected: ");
            TxHex8(test_data[i]);
            TxStr2((uint8_t*)" Got: ");
            TxHex8(read_buffer[i]);
            TxStr2((uint8_t*)"\r\n");
            match = false;
            break;
        }
    }
    
    if(match) {
        TxStr2((uint8_t*)"✓ All data verified successfully!\r\n");
        TxStr2((uint8_t*)"Read data: ");
        for(uint16_t i = 0; i < length; i++) {
            TxHex8(read_buffer[i]);
            Tx2(' ');
        }
        TxStr2((uint8_t*)"\r\n");
    }
}

void AT25_TestErase(uint32_t address) {
    uint8_t buffer[16];
    uint32_t sector_addr = address & ~(AT25_SECTOR_SIZE - 1);
    
    TxStr2((uint8_t*)"\r\n=== Testing Erase ===\r\n");
    TxLabelHex32((char*)"Sector address to erase: ", sector_addr);
    
    // Используем адрес внутри сектора для теста
    uint32_t test_address = sector_addr + 0x100;  // Смещение внутри сектора
    
    // Сначала стираем сектор, чтобы он был чистым
    TxStr2((uint8_t*)"Pre-erasing sector...\r\n");
    if(!AT25_SectorErase(sector_addr)) {
        TxStr2((uint8_t*)"✗ Pre-erase failed!\r\n");
        return;
    }
    TxStr2((uint8_t*)"✓ Sector pre-erased\r\n");
    
    uint8_t test_pattern[16];
    for(int i = 0; i < 16; i++) {
        test_pattern[i] = i + 0xAA;
    }
    
    TxStr2((uint8_t*)"Writing test pattern at address: ");
    TxHex32(test_address);
    TxStr2((uint8_t*)"\r\n");
    
    if(!AT25_WriteBytes(test_address, test_pattern, 16)) {
        TxStr2((uint8_t*)"✗ Write failed!\r\n");
        return;
    }
    
    // Проверяем что данные записались
    AT25_ReadData(test_address, buffer, 16);
    bool match = true;
    for(int i = 0; i < 16; i++) {
        if(buffer[i] != test_pattern[i]) {
            match = false;
            TxStr2((uint8_t*)"Mismatch at offset ");
            TxHex8(i);
            TxStr2((uint8_t*)" - Expected: ");
            TxHex8(test_pattern[i]);
            TxStr2((uint8_t*)" Got: ");
            TxHex8(buffer[i]);
            TxStr2((uint8_t*)"\r\n");
            break;
        }
    }
    
    if(!match) {
        TxStr2((uint8_t*)"✗ Write verification failed!\r\n");
        return;
    }
    TxStr2((uint8_t*)"✓ Test pattern written and verified\r\n");
    
    // Показываем данные до стирания
    TxStr2((uint8_t*)"Data before erase: ");
    for(int i = 0; i < 16; i++) {
        TxHex8(buffer[i]);
        Tx2(' ');
    }
    TxStr2((uint8_t*)"\r\n");
    
    // Стираем сектор
    TxStr2((uint8_t*)"Erasing sector...\r\n");
    if(!AT25_SectorErase(sector_addr)) {
        TxStr2((uint8_t*)"✗ Erase failed!\r\n");
        return;
    }
    TxStr2((uint8_t*)"✓ Sector erased\r\n");
    
    // Проверяем что данные стерты
    AT25_ReadData(test_address, buffer, 16);
    bool erased = true;
    for(int i = 0; i < 16; i++) {
        if(buffer[i] != 0xFF) {
            erased = false;
            TxStr2((uint8_t*)"Byte not erased at offset ");
            TxHex8(i);
            TxStr2((uint8_t*)" - Value: ");
            TxHex8(buffer[i]);
            TxStr2((uint8_t*)"\r\n");
            break;
        }
    }
    
    if(erased) {
        TxStr2((uint8_t*)"✓ Erase successful! All bytes are 0xFF\r\n");
        TxStr2((uint8_t*)"Data after erase: ");
        for(int i = 0; i < 16; i++) {
            TxHex8(buffer[i]);
            Tx2(' ');
        }
        TxStr2((uint8_t*)"\r\n");
    } else {
        TxStr2((uint8_t*)"✗ Erase verification failed!\r\n");
    }
}
void AT25_ForceUnprotect(void) {
    TxStr2((uint8_t*)"\r\n=== Force Unprotect ===\r\n");
    TxStr2((uint8_t*)"Note: WP pin must be HIGH to modify status register\r\n");
    
    // Проверяем состояние WP пина (он должен быть HIGH)
    TxStr2((uint8_t*)"Checking WP pin (should be connected to VCC)...\r\n");
    
    // Пытаемся записать 0x00 в статус регистр
    for(int attempt = 0; attempt < 3; attempt++) {
        TxStr2((uint8_t*)"Attempt ");
        TxHex8(attempt + 1);
        TxStr2((uint8_t*)" to write status...\r\n");
        
        AT25_WriteEnable();
        
        // Небольшая задержка
        for(volatile int i = 0; i < 1000; i++);
        
        // Проверяем WEL бит
        if(!(AT25_ReadStatus() & AT25_STATUS_WRITE_ENABLE)) {
            TxStr2((uint8_t*)"WEL bit not set, retrying...\r\n");
            continue;
        }
        
        SPI_CS_Low();
        SPI_TransferByte(AT25_CMD_WRITE_STATUS);
        SPI_TransferByte(0x00);  // Сбрасываем все биты защиты
        SPI_CS_High();
        
        // Ждем завершения
        for(volatile int i = 0; i < 10000; i++);
        
        uint8_t new_status = AT25_ReadStatus();
        TxLabelHex32((char*)"Status after attempt: ", new_status);
        
        if((new_status & 0x1C) == 0) {  // BP0, BP1, BP2 все сброшены
            TxStr2((uint8_t*)"✓ Protection bits cleared successfully!\r\n");
            return;
        }
    }
    
    TxStr2((uint8_t*)"✗ Failed to clear protection bits\r\n");
    TxStr2((uint8_t*)"Possible reasons:\r\n");
    TxStr2((uint8_t*)"  - WP pin is LOW (needs to be HIGH)\r\n");
    TxStr2((uint8_t*)"  - Chip is in hardware protect mode\r\n");
    TxStr2((uint8_t*)"  - Status register is OTP locked\r\n");
}
void AT25_TestAll(void) {
    TxStr2((uint8_t*)"\r\n========================================\r\n");
    TxStr2((uint8_t*)"Starting AT25DF321A Full Test Suite\r\n");
    TxStr2((uint8_t*)"========================================\r\n");
    
    AT25_TestReadID();
    
    uint8_t test_data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
    AT25_TestWriteRead(0x000000, test_data, sizeof(test_data));
    
    uint8_t page_test[260];
    for(int i = 0; i < 260; i++) {
        page_test[i] = i & 0xFF;
    }
    AT25_TestWriteRead(AT25_PAGE_SIZE - 10, page_test, 20);
    
    AT25_TestErase(0x000100);
    
    TxStr2((uint8_t*)"\r\n=== Final Status ===\r\n");
    uint8_t final_status = AT25_ReadStatus();
    TxLabelHex32((char*)"Status Register: ", final_status);
    
    TxStr2((uint8_t*)"\r\n========================================\r\n");
    TxStr2((uint8_t*)"Test Suite Completed!\r\n");
    TxStr2((uint8_t*)"========================================\r\n");
}


void AT25_WriteStatus(uint8_t status) {
    if(!spi_initialized) return;
    
    AT25_WriteEnable();
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_WRITE_STATUS);
    SPI_TransferByte(status);
    SPI_CS_High();
    
    AT25_WaitForReady(100);
}

void AT25_Unprotect(void) {
    TxStr2((uint8_t*)"\r\n=== Removing Write Protection ===\r\n");
    
    // Читаем текущий статус
    uint8_t status = AT25_ReadStatus();
    TxLabelHex32((char*)"Current Status: ", status);
    
    // Сбрасываем биты защиты (BP0, BP1, BP2)
    // Оставляем только бит 7 (SRWD) если нужно
    uint8_t new_status = status & ~(AT25_STATUS_BP0 | AT25_STATUS_BP1 | AT25_STATUS_BP2);
    
    TxLabelHex32((char*)"New Status: ", new_status);
    
    // Записываем новый статус
    AT25_WriteStatus(new_status);
    
    // Проверяем
    status = AT25_ReadStatus();
    TxLabelHex32((char*)"Verification Status: ", status);
    
    if(!(status & (AT25_STATUS_BP0 | AT25_STATUS_BP1 | AT25_STATUS_BP2))) {
        TxStr2((uint8_t*)"✓ Write protection removed successfully!\r\n");
    } else {
        TxStr2((uint8_t*)"✗ Failed to remove write protection!\r\n");
    }
}