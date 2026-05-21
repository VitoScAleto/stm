#include "../inc/swd.h"
#include <stdint.h>
#include <stdio.h>
#include "i2c_at24c256.c"
#include "../inc/oled.h"

// ================= КОНФИГУРАЦИЯ =================
#define SWCLK_PORT   GPIOA
#define SWCLK_PIN    8
#define SWDIO_PORT   GPIOA
#define SWDIO_PIN    9
#define NRST_PORT    GPIOB
#define NRST_PIN     0
#define LED_PORT     GPIOC
#define LED_PIN      13

// Адреса регистров отладки
#define DP_RDBUFF    0x0C
#define DP_ABORT     0x00
#define DP_IDCODE    0x00
#define DP_CTRLSTAT  0x04
#define DP_SELECT    0x08

#define AP_CSW       0x00
#define AP_TAR       0x04
#define AP_DRW       0x0C

// Флаги DP_CTRLSTAT
#define CSYSPWRUPACK (1u << 31)
#define CDBGPWRUPACK (1u << 29)
#define CSYSPWRUPREQ (1u << 30)
#define CDBGPWRUPREQ (1u << 28)

// Flash target STM32F1
#define FLASH_KEYR_ADDR     0x40022004u
#define FLASH_SR_ADDR       0x4002200Cu
#define FLASH_CR_ADDR       0x40022010u
#define FLASH_AR_ADDR       0x40022014u

#define FLASH_SR_BSY        (1u << 0)
#define FLASH_SR_PGERR      (1u << 2)
#define FLASH_SR_WRPRTERR   (1u << 4)

#define FLASH_CR_PG         (1u << 0)
#define FLASH_CR_PER        (1u << 1)
#define FLASH_CR_STRT       (1u << 6)
#define FLASH_CR_LOCK       (1u << 7)

#define FLASH_KEY1_VALUE    0x45670123u
#define FLASH_KEY2_VALUE    0xCDEF89ABu

// AP CSW
#define AP_CSW_32BIT        0x22000012u
#define AP_CSW_16BIT        0x22000011u

// GPIO macros
#define SWDIO_HIGH()  GPIOA->BSRR = (1u << SWDIO_PIN)
#define SWDIO_LOW()   GPIOA->BSRR = (1u << (SWDIO_PIN + 16))
#define SWCLK_HIGH()  GPIOA->BSRR = (1u << SWCLK_PIN)
#define SWCLK_LOW()   GPIOA->BSRR = (1u << (SWCLK_PIN + 16))
#define NRST_HIGH()   GPIOB->BSRR = (1u << NRST_PIN)
#define NRST_LOW()    GPIOB->BSRR = (1u << (NRST_PIN + 16))

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000u);
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles) {
    }
}

static void led_toggle(void)
{
    LED_PORT->ODR ^= (1u << LED_PIN);
}

static void swd_gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    SWCLK_PORT->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    SWCLK_PORT->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;

    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    SWDIO_PORT->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;

    NRST_PORT->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    NRST_PORT->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0;

    LED_PORT->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    LED_PORT->CRH |= GPIO_CRH_MODE13_1;

    SWCLK_LOW();
    NRST_PORT->BSRR = (1u << NRST_PIN);
    LED_PORT->BRR = (1u << LED_PIN);
}

static void swd_io_dir_output(void)
{
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9);
    SWDIO_PORT->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
}

static void swd_io_dir_input(void)
{
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    SWDIO_PORT->CRH |= GPIO_CRH_CNF9_1;
    SWDIO_PORT->ODR |= (1u << SWDIO_PIN);
}

static void swd_write_bit(uint8_t bit)
{
    if (bit) {
        SWDIO_HIGH();
    } else {
        SWDIO_LOW();
    }

    SWCLK_LOW();
    delay_us(1);

    SWCLK_HIGH();
    delay_us(1);
}

static uint8_t swd_read_bit(void)
{
    uint8_t bit;

    SWCLK_LOW();
    delay_us(1);

    bit = (SWDIO_PORT->IDR >> SWDIO_PIN) & 1u;

    SWCLK_HIGH();
    delay_us(1);

    return bit;
}

static void swd_turnaround(void)
{
    SWCLK_LOW();
    delay_us(1);

    SWCLK_HIGH();
    delay_us(1);
}

static uint32_t swd_transfer(uint8_t req, uint32_t data)
{
    uint32_t val = 0;
    uint8_t ack;

    swd_io_dir_output();
    for (int i = 0; i < 8; i++) {
        swd_write_bit((req >> i) & 1u);
    }

    swd_io_dir_input();
    swd_turnaround();

    ack = 0;
    for (int i = 0; i < 3; i++) {
        ack |= (swd_read_bit() << i);
    }

    (void)ack;

    if (req & (1u << 2)) {
        uint8_t parity_calc = 0;

        for (int i = 0; i < 32; i++) {
            uint8_t b = swd_read_bit();
            val |= ((uint32_t)b << i);
            parity_calc ^= b;
        }

        uint8_t parity_bit = swd_read_bit();
        if (parity_calc != parity_bit) {
            Tx2('E');
        }

        swd_io_dir_output();
        swd_turnaround();
    } else {
        swd_turnaround();
        swd_io_dir_output();

        for (int i = 0; i < 32; i++) {
            swd_write_bit((data >> i) & 1u);
        }

        uint8_t parity = __builtin_popcount(data) & 1u;
        swd_write_bit(parity);

        swd_write_bit(0);
    }

    return val;
}

static void swd_line_reset(void)
{
    swd_io_dir_output();
    SWDIO_HIGH();

    for (int i = 0; i < 55; i++) {
        SWCLK_LOW();
        delay_us(1);

        SWCLK_HIGH();
        delay_us(1);
    }
}

static void swd_jtag_to_swd(void)
{
    uint16_t switch_sequence = 0xE79E;

    for (int i = 0; i < 16; i++) {
        if (switch_sequence & 0x01u) {
            SWDIO_HIGH();
        } else {
            SWDIO_LOW();
        }

        SWCLK_LOW();
        delay_us(1);

        SWCLK_HIGH();
        delay_us(1);

        switch_sequence >>= 1;
    }
}

static uint8_t swd_make_request(uint8_t apndp, uint8_t rnw, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 1u;
    uint8_t a3 = (addr >> 3) & 1u;
    uint8_t parity = (apndp ^ rnw ^ a2 ^ a3) & 1u;

    uint8_t req = 0;
    req |= (1u << 0);
    req |= (apndp << 1);
    req |= (rnw << 2);
    req |= (a2 << 3);
    req |= (a3 << 4);
    req |= (parity << 5);
    req |= (0u << 6);
    req |= (1u << 7);

    return req;
}

static uint32_t swd_read_idcode(void)
{
    uint8_t req =
        (1u << 0) |
        (0u << 1) |
        (1u << 2) |
        (0u << 3) |
        (0u << 4) |
        (1u << 5) |
        (0u << 6) |
        (1u << 7);

    return swd_transfer(req, 0);
}

static uint32_t swd_read_dp(uint8_t addr)
{
    uint8_t req = swd_make_request(0, 1, addr);
    return swd_transfer(req, 0);
}

static void swd_write_dp(uint8_t addr, uint32_t data)
{
    uint8_t req = swd_make_request(0, 0, addr);
    swd_transfer(req, data);
}

static void swd_write_ap(uint8_t ap, uint8_t addr, uint32_t data)
{
    swd_write_dp(DP_SELECT, ((uint32_t)ap << 24) | (addr & 0xF0u));

    uint8_t req = swd_make_request(1, 0, addr);
    swd_transfer(req, data);
}

static uint32_t swd_read_ap(uint8_t ap, uint8_t addr)
{
    swd_write_dp(DP_SELECT, ((uint32_t)ap << 24) | (addr & 0xF0u));

    uint8_t req = swd_make_request(1, 1, addr);
    swd_transfer(req, 0);

    req = swd_make_request(0, 1, DP_RDBUFF);
    return swd_transfer(req, 0);
}

static uint32_t swd_mem_read(uint32_t addr)
{
    swd_write_ap(0, AP_TAR, addr);
    return swd_read_ap(0, AP_DRW);
}

static void swd_mem_write(uint32_t addr, uint32_t data)
{
    swd_write_ap(0, AP_TAR, addr);
    swd_write_ap(0, AP_DRW, data);
}

static int swd_init_debug(void)
{
    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();

    SWDIO_LOW();
    swd_turnaround();
    swd_turnaround();

    uint32_t id = swd_read_idcode();
    if (id == 0xFFFFFFFFu || id == 0u) {
        return 0;
    }

    swd_write_dp(DP_CTRLSTAT, CSYSPWRUPREQ | CDBGPWRUPREQ);

    swd_write_dp(DP_SELECT, 0x00000000u);
    swd_write_ap(0, AP_CSW, AP_CSW_32BIT);

    return 1;
}

static void swd_set_csw(uint32_t csw)
{
    swd_write_ap(0, AP_CSW, csw);
}

static void swd_mem_write16(uint32_t addr, uint16_t data)
{
    uint32_t payload;

    swd_write_ap(0, AP_CSW, AP_CSW_16BIT);
    swd_write_ap(0, AP_TAR, addr);

    if (addr & 0x2u) {
        payload = ((uint32_t)data) << 16;
    } else {
        payload = (uint32_t)data;
    }

    swd_write_ap(0, AP_DRW, payload);
    swd_write_ap(0, AP_CSW, AP_CSW_32BIT);
}

static uint16_t swd_mem_read16(uint32_t addr)
{
    uint32_t v;

    swd_set_csw(AP_CSW_16BIT);
    swd_write_ap(0, AP_TAR, addr);
    v = swd_read_ap(0, AP_DRW);
    swd_set_csw(AP_CSW_32BIT);

    if (addr & 0x2u) {
        return (uint16_t)(v >> 16);
    } else {
        return (uint16_t)(v & 0xFFFFu);
    }
}

static int flash_wait_ready(void)
{
    uint32_t sr;

    do {
        sr = swd_mem_read(FLASH_SR_ADDR);
    } while (sr & FLASH_SR_BSY);

    if (sr & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
        return 0;
    }

    return 1;
}

static void flash_unlock_target(void)
{
    swd_mem_write(FLASH_KEYR_ADDR, FLASH_KEY1_VALUE);
    swd_mem_write(FLASH_KEYR_ADDR, FLASH_KEY2_VALUE);
}

static void flash_lock_target(void)
{
    uint32_t cr = swd_mem_read(FLASH_CR_ADDR);
    swd_mem_write(FLASH_CR_ADDR, cr | FLASH_CR_LOCK);
}

static int flash_erase_page(uint32_t page_addr)
{
    if (!flash_wait_ready()) {
        return 0;
    }

    swd_mem_write(FLASH_CR_ADDR, FLASH_CR_PER);
    swd_mem_write(FLASH_AR_ADDR, page_addr);
    swd_mem_write(FLASH_CR_ADDR, FLASH_CR_PER | FLASH_CR_STRT);

    if (!flash_wait_ready()) {
        swd_mem_write(FLASH_CR_ADDR, 0);
        return 0;
    }

    swd_mem_write(FLASH_CR_ADDR, 0);
    return 1;
}

static int flash_program_halfword(uint32_t addr, uint16_t data)
{
    if (addr & 1u) {
        return 0;
    }

    if (!flash_wait_ready()) {
        return 0;
    }

    swd_mem_write(FLASH_CR_ADDR, FLASH_CR_PG);
    swd_mem_write16(addr, data);

    if (!flash_wait_ready()) {
        swd_mem_write(FLASH_CR_ADDR, 0);
        return 0;
    }

    swd_mem_write(FLASH_CR_ADDR, 0);
    return 1;
}

static int flash_program_word32(uint32_t addr, uint32_t data)
{
    if (addr & 3u) {
        return 0;
    }

    if (!flash_program_halfword(addr, (uint16_t)(data & 0xFFFFu))) {
        return 0;
    }
    if (!flash_program_halfword(addr + 2u, (uint16_t)(data >> 16))) {
        return 0;
    }

    return 1;
}

static int flash_program_block(uint32_t base_addr, const uint32_t *data, uint32_t word_count)
{
    for (uint32_t i = 0; i < word_count; i++) {
        if ((i * 4u) % 0x400u == 0u) {
            flash_erase_page(base_addr + i * 4u);
        }

        if (!flash_program_word32(base_addr + i * 4u, data[i])) {
            return 0;
        }
    }

    return 1;
}

int check_target(){
    uint32_t id = swd_read_idcode();
    return !(id == 0xFFFFFFFF || id == 0x00000000);
}

#define FLASH_BASE_ADDR 0x08000000U
#define FLASH_PAGE_SIZE 1024U

static void flash_erase_all_needed(uint32_t base_addr, uint32_t size)
{
    uint32_t addr = base_addr;
    uint32_t end  = base_addr + size;

    while (addr < end)
    {
        flash_erase_page(addr);
        addr += FLASH_PAGE_SIZE;
    }
}


#include "../inc/at25df321a_firmware.h"
#include "../inc/uart_firmware_handler.h"
#include "../inc/at25df321a.h"
#include "../inc/stm32f1xx.h"
#include <stdint.h>
#include <string.h>

void UART_SendByte(uint8_t data) {
    while(!(USART2->SR & USART_SR_TXE));
    USART2->DR = data;
}

void UART_SendString(const char* str) {
    while(*str) UART_SendByte((uint8_t)*str++);
}

uint8_t UART_ReceiveByte(void) {
    while(!(USART2->SR & USART_SR_RXNE));
    return (uint8_t)USART2->DR;
}

void SystemClock_Init(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

int main(void) {
    SystemClock_Init();
    USART2_Init();

    UART_SendString("\r\n========================================\r\n");
    UART_SendString("System Started at 72MHz\r\n");
    UART_SendString("========================================\r\n");

    AT25_Init();
    AT25_ForceUnprotect();
    UART_Firmware_Init();

    UART_SendString("System ready. Waiting for commands...\r\n");
    UART_SendString("========================================\r\n");

    while(1) {
        if(USART2->SR & USART_SR_RXNE) {
            uint8_t cmd = (uint8_t)USART2->DR;
            UART_Firmware_ProcessCommand(cmd);
        }
    }
}
