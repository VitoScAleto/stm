
#include "at25df321a.h"

static uint8_t spi_initialized = 0;

// Прототипы статических функций
static void SPI_Delay(void);
static void SPI_CS_Low(void);
static void SPI_CS_High(void);
static uint8_t SPI_TransferByte(uint8_t data);
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
    
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if(--timeout == 0) {
            return 0xFF;
        }
    }
    
    *((volatile uint8_t*)&SPI1->DR) = data;
    
    timeout = 100000;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if(--timeout == 0) {
            return 0xFF;
        }
    }
    
    return (uint8_t)SPI1->DR;
}

static void SPI1_Init(void) {
    // Включаем тактирование
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
    
    for(volatile int i = 0; i < 1000; i++);
    
    // Сброс SPI1
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    for(volatile int i = 0; i < 100; i++);
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    
    // Настройка пинов SPI
    // PA4 - CS
    GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF4_0;
    GPIOA->CRL &= ~GPIO_CRL_CNF4_1;
    GPIOA->BSRR = GPIO_BSRR_BS4;
    
    // PA5 - SCK
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1;
    GPIOA->CRL |= GPIO_CRL_CNF5_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
    
    // PA6 - MISO
    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_CNF6_0;
    GPIOA->CRL &= ~GPIO_CRL_MODE6;
    
    // PA7 - MOSI
    GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1;
    GPIOA->CRL |= GPIO_CRL_CNF7_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF7_0;
    
    // Настройка SPI1
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SSM;
    
    SPI1->CR1 |= SPI_CR1_BR_0;
    SPI1->CR1 &= ~SPI_CR1_BR_1;
    SPI1->CR1 &= ~SPI_CR1_BR_2;
    
    SPI1->CR1 &= ~SPI_CR1_CPOL;
    SPI1->CR1 &= ~SPI_CR1_CPHA;
    SPI1->CR1 &= ~SPI_CR1_DFF;
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
    
    SPI1->CR1 |= SPI_CR1_SPE;
    
    if(SPI1->CR1 & SPI_CR1_SPE) {
        spi_initialized = 1;
    }
}

// Публичные функции
void AT25_Init(void) {
    SPI1_Init();
    
    if(!spi_initialized) {
        return;
    }
    
    // Выход из deep power-down
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_RESUME);
    SPI_CS_High();
    
    for(volatile int i = 0; i < 10000; i++);
}

uint8_t AT25_ReadStatus(void) {
    if(!spi_initialized) {
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
        for(volatile int i = 0; i < 100; i++);
    }
    
    return false;
}

void AT25_WriteEnable(void) {
    if(!spi_initialized) return;
    
    SPI_CS_Low();
    SPI_TransferByte(AT25_CMD_WRITE_ENABLE);
    SPI_CS_High();
    
    for(volatile int i = 0; i < 1000; i++);
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
    
    if(length > AT25_PAGE_SIZE || (address + length) > ((address & ~(AT25_PAGE_SIZE - 1)) + AT25_PAGE_SIZE)) {
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    
    AT25_WriteEnable();
    
    if(!(AT25_ReadStatus() & AT25_STATUS_WRITE_ENABLE)) {
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
    
    return AT25_WaitForReady(100);
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
        return false;
    }
    
    if(!AT25_WaitForReady(100)) return false;
    AT25_WriteEnable();
    
    if(!(AT25_ReadStatus() & AT25_STATUS_WRITE_ENABLE)) {
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
    uint8_t status = AT25_ReadStatus();
    uint8_t new_status = status & ~(AT25_STATUS_BP0 | AT25_STATUS_BP1 | AT25_STATUS_BP2);
    AT25_WriteStatus(new_status);
}

void AT25_ForceUnprotect(void) {
    for(int attempt = 0; attempt < 3; attempt++) {
        AT25_WriteEnable();
        
        for(volatile int i = 0; i < 1000; i++);
        
        if(!(AT25_ReadStatus() & AT25_STATUS_WRITE_ENABLE)) {
            continue;
        }
        
        SPI_CS_Low();
        SPI_TransferByte(AT25_CMD_WRITE_STATUS);
        SPI_TransferByte(0x00);
        SPI_CS_High();
        
        for(volatile int i = 0; i < 10000; i++);
        
        uint8_t new_status = AT25_ReadStatus();
        if((new_status & 0x1C) == 0) {
            return;
        }
    }
}