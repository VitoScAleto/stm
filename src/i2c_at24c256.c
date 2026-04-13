#include "../inc/swd.h"
#include <stdint.h>

#define I2C_TIMEOUT    100000UL
#define AT24C256_I2C                I2C1
#define AT24C256_ADDR_7BIT          0x50
#define AT24C256_PAGE_SIZE          64
#define AT24C256_TOTAL_SIZE         32768

void I2C1_Init_AT24C256(void);

uint8_t AT24C256_WriteByte(uint16_t mem_addr, uint8_t data);
uint8_t AT24C256_ReadByte(uint16_t mem_addr, uint8_t *data);

uint8_t AT24C256_WritePage(uint16_t mem_addr, const uint8_t *data, uint16_t len);
uint8_t AT24C256_ReadBuffer(uint16_t mem_addr, uint8_t *data, uint16_t len);

// PB6 - SCL PB7 - SDA

static uint8_t i2c_wait_flag_set(volatile uint32_t *reg, uint16_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((*reg) & flag) == 0U)
    {
        if (--timeout == 0U) return 0;
    }
    return 1;
}

static uint8_t i2c_wait_flag_clear(volatile uint32_t *reg, uint16_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((*reg) & flag) != 0U)
    {
        if (--timeout == 0U) return 0;
    }
    return 1;
}

static void delay_simple(volatile uint32_t t)
{
    while (t--) __NOP();
}

void I2C1_Init_AT24C256(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* I2C1 без remap */
    AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;

    GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->CRL |=  ((0xF << (6 * 4)) | (0xF << (7 * 4)));

    /* Сброс I2C */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 36;
    I2C1->CCR = 180;

    I2C1->TRISE = 37;

    I2C1->CR1 |= I2C_CR1_ACK;

    I2C1->CR1 |= I2C_CR1_PE;
}

static uint8_t i2c_start(void)
{
    /* Ждём BUSY=0 */
    if (!i2c_wait_flag_clear(&I2C1->SR2, I2C_SR2_BUSY))
        return 0;

    I2C1->CR1 |= I2C_CR1_START;

    /* SB */
    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_SB))
        return 0;

    return 1;
}

static void i2c_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static uint8_t i2c_send_addr(uint8_t addr8)
{
    I2C1->DR = addr8;

    /* Ждём ADDR */
    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_ADDR))
        return 0;

    /* Очистка ADDR: чтение SR1, потом SR2 */
    volatile uint16_t tmp;
    tmp = I2C1->SR1;
    tmp = I2C1->SR2;
    (void)tmp;

    return 1;
}

static uint8_t i2c_write_data(uint8_t data)
{
    /* Ждём TXE */
    while((I2C1->SR1 & I2C_SR1_TXE) == 0) ;
    I2C1->DR = data;

    /* Ждём BTF или TXE */

    while((I2C1->SR1 & I2C_SR1_BTF) == 0) ;
    return 1;
}

static uint8_t i2c_read_data_ack(uint8_t *data)
{
    I2C1->CR1 |= I2C_CR1_ACK;

    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE))
        return 0;

    *data = (uint8_t)I2C1->DR;
    return 1;
}

static uint8_t i2c_read_data_nack(uint8_t *data)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;

    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE))
        return 0;

    *data = (uint8_t)I2C1->DR;
    return 1;
}

/* Ожидание завершения внутренней записи EEPROM */
static uint8_t at24c256_wait_ready(void)
{
    uint32_t tries = 1000;

    while (tries--)
    {
        if (!i2c_start())
            continue;

        /*
            EEPROM после записи не отвечает ACK,
            пока внутренний цикл записи не закончится.
        */
        I2C1->DR = (AT24C256_ADDR_7BIT << 1); /* write */

        uint32_t timeout = I2C_TIMEOUT;
        while (((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0U) && (--timeout));

        if (timeout == 0U)
        {
            i2c_stop();
            return 0;
        }

        if (I2C1->SR1 & I2C_SR1_ADDR)
        {
            volatile uint16_t tmp;
            tmp = I2C1->SR1;
            tmp = I2C1->SR2;
            (void)tmp;

            i2c_stop();
            return 1;
        }

        if (I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            i2c_stop();
            delay_simple(2000);
        }
    }

    return 0;
}

uint8_t AT24C256_WriteByte(uint16_t mem_addr, uint8_t data)
{
    if (!i2c_start()) return 0;
    if (!i2c_send_addr((AT24C256_ADDR_7BIT << 1) | 0)) { i2c_stop(); return 0; }

    if (!i2c_write_data((uint8_t)(mem_addr >> 8)))      { i2c_stop(); return 0; }
    if (!i2c_write_data((uint8_t)(mem_addr & 0xFF)))    { i2c_stop(); return 0; }
    if (!i2c_write_data(data))                          { i2c_stop(); return 0; }

    i2c_stop();

    return at24c256_wait_ready();
}

uint8_t AT24C256_ReadByte(uint16_t mem_addr, uint8_t *data)
{
    if (data == 0) return 0;

    /* Сначала передаём адрес памяти */
    if (!i2c_start()) return 0;
    if (!i2c_send_addr((AT24C256_ADDR_7BIT << 1) | 0)) { i2c_stop(); return 0; }

    if (!i2c_write_data((uint8_t)(mem_addr >> 8)))     { i2c_stop(); return 0; }
    if (!i2c_write_data((uint8_t)(mem_addr & 0xFF)))   { i2c_stop(); return 0; }

    /* Повторный старт и чтение */
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_SB))    { i2c_stop(); return 0; }

    I2C1->DR = (AT24C256_ADDR_7BIT << 1) | 1;

    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_ADDR))  { i2c_stop(); return 0; }

    /* Для чтения 1 байта: ACK=0 до очистки ADDR */
    I2C1->CR1 &= ~I2C_CR1_ACK;

    volatile uint16_t tmp;
    tmp = I2C1->SR1;
    tmp = I2C1->SR2;
    (void)tmp;

    i2c_stop();

    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE))  return 0;

    *data = (uint8_t)I2C1->DR;

    /* Вернём ACK обратно */
    I2C1->CR1 |= I2C_CR1_ACK;

    return 1;
}

uint8_t AT24C256_ReadBuffer(uint16_t mem_addr, uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0)) return 0;

    /* Установка внутреннего адреса EEPROM */
    if (!i2c_start()) return 0;
    if (!i2c_send_addr((AT24C256_ADDR_7BIT << 1) | 0)) { i2c_stop(); return 0; }

    if (!i2c_write_data((uint8_t)(mem_addr >> 8)))     { i2c_stop(); return 0; }
    if (!i2c_write_data((uint8_t)(mem_addr & 0xFF)))   { i2c_stop(); return 0; }

    /* Повторный старт */
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_SB))    { i2c_stop(); return 0; }

    I2C1->DR = (AT24C256_ADDR_7BIT << 1) | 1;
    if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_ADDR))  { i2c_stop(); return 0; }

    volatile uint16_t tmp;

    if (len == 1)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        tmp = I2C1->SR1;
        tmp = I2C1->SR2;
        (void)tmp;

        i2c_stop();

        if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE)) return 0;
        data[0] = (uint8_t)I2C1->DR;
    }
    else
    {
        I2C1->CR1 |= I2C_CR1_ACK;
        tmp = I2C1->SR1;
        tmp = I2C1->SR2;
        (void)tmp;

        for (uint16_t i = 0; i < len; i++)
        {
            if (i == (len - 1))
            {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                i2c_stop();

                if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE)) return 0;
                data[i] = (uint8_t)I2C1->DR;
            }
            else
            {
                if (!i2c_wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE)) return 0;
                data[i] = (uint8_t)I2C1->DR;
            }
        }
    }

    I2C1->CR1 |= I2C_CR1_ACK;
    return 1;
}

uint8_t AT24C256_WritePage(uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0)) return 0;
    if (len > AT24C256_PAGE_SIZE) return 0;

    /*
        Нельзя переходить через границу страницы.
        Например, если mem_addr = 60, то максимум можно записать 4 байта.
    */
    uint16_t page_offset = mem_addr % AT24C256_PAGE_SIZE;
    if ((page_offset + len) > AT24C256_PAGE_SIZE) return 0;

    if (!i2c_start()) return 0;
    if (!i2c_send_addr((AT24C256_ADDR_7BIT << 1) | 0)) { i2c_stop(); return 0; }

    if (!i2c_write_data((uint8_t)(mem_addr >> 8)))     { i2c_stop(); return 0; }
    if (!i2c_write_data((uint8_t)(mem_addr & 0xFF)))   { i2c_stop(); return 0; }

    for (uint16_t i = 0; i < len; i++)
    {
        if (!i2c_write_data(data[i]))
        {
            i2c_stop();
            return 0;
        }
    }

    i2c_stop();

    return at24c256_wait_ready();
}

uint8_t AT24C256_WriteBuffer(uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0)
        return 0;

    /* Проверка выхода за пределы памяти EEPROM */
    if ((uint32_t)mem_addr + len > AT24C256_TOTAL_SIZE)
        return 0;

    while (len > 0)
    {
        uint16_t page_offset = mem_addr % AT24C256_PAGE_SIZE;
        uint16_t page_space  = AT24C256_PAGE_SIZE - page_offset;
        uint16_t chunk       = (len < page_space) ? len : page_space;

        if (!AT24C256_WritePage(mem_addr, data, chunk))
            return 0;

        mem_addr += chunk;
        data     += chunk;
        len      -= chunk;
    }

    return 1;
}