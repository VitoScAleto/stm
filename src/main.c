#include "../inc/swd.h"
#include <stdint.h>
#include <stdio.h>
#include "i2c_at24c256.c"
#include "../inc/oled.h"

#include "../inc/at25df321a_firmware.h"
#include "../inc/uart_firmware_handler.h"
#include "../inc/at25df321a.h"
#include "../inc/stm32f1xx.h"
#include <stdint.h>
#include <string.h>
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


void UART_SendByte(uint8_t data);

static int swd_prepare_target_for_program(uint32_t firmware_size)
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

static void UART_SendU32(uint32_t value)
{
    UART_SendByte((uint8_t)(value & 0xFFu));
    UART_SendByte((uint8_t)((value >> 8) & 0xFFu));
    UART_SendByte((uint8_t)((value >> 16) & 0xFFu));
    UART_SendByte((uint8_t)((value >> 24) & 0xFFu));
}

static int UART_ReceiveExactLocal(uint8_t *buffer, uint32_t length, uint32_t timeout_loops)
{
    uint32_t received = 0;
    uint32_t timeout = timeout_loops;

    while (received < length) {
        if (USART2->SR & USART_SR_RXNE) {
            buffer[received++] = (uint8_t)USART2->DR;
            timeout = timeout_loops;
        } else {
            if (timeout-- == 0u) {
                return 0;
            }
        }
    }

    return 1;
}

static void CMD_DirectSwdCheckTarget(void)
{
    swd_gpio_init();
    dwt_init();

    NRST_LOW();
    delay_us(5000);
    NRST_HIGH();
    delay_us(50000);

    if (swd_init_debug() && check_target()) {
        UART_SendByte(RESP_OK);
    } else {
        UART_SendByte(RESP_ERR);
    }
}

static void CMD_DirectSwdProgramFile(void)
{
    uint8_t size_bytes[4];

    if (!UART_ReceiveExactLocal(size_bytes, 4, 3000000u)) {
        UART_SendByte(RESP_ERR);
        return;
    }

    uint32_t total_words = ((uint32_t)size_bytes[0]) |
                           ((uint32_t)size_bytes[1] << 8) |
                           ((uint32_t)size_bytes[2] << 16) |
                           ((uint32_t)size_bytes[3] << 24);

    if (total_words == 0u) {
        UART_SendByte(RESP_ERR);
        return;
    }

    uint32_t firmware_size = total_words * 4u;

    if (!swd_prepare_target_for_program(firmware_size)) {
        UART_SendByte(RESP_ERR);
        return;
    }

    UART_SendByte(RESP_OK);

    for (uint32_t i = 0; i < total_words; i++) {
        uint8_t word_bytes[4];

        if (!UART_ReceiveExactLocal(word_bytes, 4, 3000000u)) {
            flash_lock_target();
            UART_SendByte(RESP_ERR);
            return;
        }

        uint32_t word = ((uint32_t)word_bytes[0]) |
                        ((uint32_t)word_bytes[1] << 8) |
                        ((uint32_t)word_bytes[2] << 16) |
                        ((uint32_t)word_bytes[3] << 24);

        if (!flash_program_word32(FLASH_BASE_ADDR + i * 4u, word)) {
            flash_lock_target();
            UART_SendByte(RESP_ERR);
            return;
        }

        UART_SendU32(word);
    }

    flash_lock_target();
}

static void CMD_DirectSwdEndProgram(void)
{
    UART_SendByte(RESP_OK);
}

// Команда 'F':
// PC -> STM32: 'F' + uint32_t index
// STM32 -> PC: K + uint32_t firmware_size
// STM32 -> PC: K после каждого 64-byte чанка
// STM32 -> PC: финальный K
static void CMD_FlashFirmwareFromAT25(void)
{
    uint8_t index_bytes[4];

    if (!UART_ReceiveExactLocal(index_bytes, 4, 3000000u)) {
        UART_SendByte(RESP_ERR);
        return;
    }

    uint32_t index = ((uint32_t)index_bytes[0]) |
                     ((uint32_t)index_bytes[1] << 8) |
                     ((uint32_t)index_bytes[2] << 16) |
                     ((uint32_t)index_bytes[3] << 24);

    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];

    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if (index >= count) {
        UART_SendByte(RESP_ERR);
        return;
    }

    uint32_t firmware_size = headers[index].size;
    uint32_t firmware_addr = addresses[index] + FIRMWARE_HEADER_SIZE;

    if (firmware_size == 0u) {
        UART_SendByte(RESP_ERR);
        return;
    }

    uint32_t program_size = (firmware_size + 3u) & ~3u;

    if (!swd_prepare_target_for_program(program_size)) {
        UART_SendByte(RESP_ERR);
        return;
    }

    UART_SendByte(RESP_OK);
    UART_SendU32(firmware_size);

    uint8_t buffer[64];
    uint32_t offset = 0;

    while (offset < firmware_size) {
        uint32_t chunk = firmware_size - offset;
        if (chunk > sizeof(buffer)) {
            chunk = sizeof(buffer);
        }

        memset(buffer, 0xFF, sizeof(buffer));
        AT25_ReadData(firmware_addr + offset, buffer, chunk);

        uint32_t chunk_program_size = (chunk + 3u) & ~3u;

        for (uint32_t i = 0; i < chunk_program_size; i += 4u) {
            uint32_t word = ((uint32_t)buffer[i]) |
                            ((uint32_t)buffer[i + 1u] << 8) |
                            ((uint32_t)buffer[i + 2u] << 16) |
                            ((uint32_t)buffer[i + 3u] << 24);

            if (!flash_program_word32(FLASH_BASE_ADDR + offset + i, word)) {
                flash_lock_target();
                UART_SendByte(RESP_ERR);
                return;
            }
        }

        offset += chunk;
        UART_SendByte(RESP_OK);
    }

    flash_lock_target();
    UART_SendByte(RESP_OK);
}



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

#define BTN_UP_PIN      10
#define BTN_DOWN_PIN    11
#define BTN_OK_PIN      12
#define BTN_BACK_PIN    13

#define BTN_UP_MASK     (1u << BTN_UP_PIN)
#define BTN_DOWN_MASK   (1u << BTN_DOWN_PIN)
#define BTN_OK_MASK     (1u << BTN_OK_PIN)
#define BTN_BACK_MASK   (1u << BTN_BACK_PIN)

typedef enum {
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_OK,
    KEY_BACK
} Key_t;

typedef enum {
    UI_MAIN = 0,
    UI_SELECT_FW,
    UI_BUSY,
    UI_MESSAGE
} UiState_t;

static UiState_t ui_state = UI_MAIN;
static uint32_t ui_menu_index = 0;
static uint32_t ui_fw_index = 0;

static void Buttons_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // PB12 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    GPIOB->CRH |= GPIO_CRH_CNF12_1;
    GPIOB->ODR |= BTN_OK_MASK;

    // PB13 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOB->CRH |= GPIO_CRH_CNF13_1;
    GPIOB->ODR |= BTN_BACK_MASK;

    // PB10 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOB->CRH |= GPIO_CRH_CNF10_1;
    GPIOB->ODR |= BTN_UP_MASK;

    // PB11 input pull-up
    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_CNF11_1;
    GPIOB->ODR |= BTN_DOWN_MASK;
}

static uint8_t Button_IsPressed(uint32_t mask)
{
    return (GPIOB->IDR & mask) == 0;
}

static Key_t Buttons_GetKey(void)
{
    static uint32_t lock = 0;

    if(lock) {
        lock--;
        return KEY_NONE;
    }

    if(Button_IsPressed(BTN_UP_MASK)) {
        lock = 70000;
        return KEY_UP;
    }

    if(Button_IsPressed(BTN_DOWN_MASK)) {
        lock = 70000;
        return KEY_DOWN;
    }

    if(Button_IsPressed(BTN_OK_MASK)) {
        lock = 70000;
        return KEY_OK;
    }

    if(Button_IsPressed(BTN_BACK_MASK)) {
        lock = 70000;
        return KEY_BACK;
    }

    return KEY_NONE;
}

static void OLED_Status(const char* a, const char* b)
{
    OLED_Clear();
    OLED_DrawString(0, 0, a, 1);
    OLED_DrawString(0, 16, b, 1);
    OLED_Update();
}

static void OLED_ShowMainMenu(void)
{
    OLED_Clear();

    OLED_DrawString(0, 0, "SWD PROGRAMMER", 1);

    OLED_DrawString(0, 16, ui_menu_index == 0 ? "> Flash AT25" : "  Flash AT25", 1);
    OLED_DrawString(0, 26, ui_menu_index == 1 ? "> Verify CRC" : "  Verify CRC", 1);
    OLED_DrawString(0, 36, ui_menu_index == 2 ? "> Memory Info" : "  Memory Info", 1);

    OLED_Update();
}

static void OLED_ShowFirmwareSelect(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    OLED_Clear();

    if(count == 0) {
        OLED_DrawString(0, 0, "No firmware", 1);
        OLED_DrawString(0, 16, "BACK", 1);
        OLED_Update();
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    char line[32];

    OLED_DrawString(0, 0, "Select firmware", 1);

    snprintf(line, sizeof(line), "%lu/%lu",
             (unsigned long)(ui_fw_index + 1),
             (unsigned long)count);
    OLED_DrawString(96, 0, line, 1);

    OLED_DrawString(0, 16, ">", 1);
    OLED_DrawString(8, 16, (char*)headers[ui_fw_index].name, 1);

    snprintf(line, sizeof(line), "Size:%lu",
             (unsigned long)headers[ui_fw_index].size);
    OLED_DrawString(0, 32, line, 1);

    snprintf(line, sizeof(line), "CRC:%08lX",
             (unsigned long)headers[ui_fw_index].crc32);
    OLED_DrawString(0, 42, line, 1);

    OLED_DrawString(0, 56, "OK=Flash BACK=Exit", 1);

    OLED_Update();
}
static int FlashFirmwareFromAT25_Index(uint32_t index, uint8_t use_oled)
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
static void OLED_DoFlashSelected(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(count == 0) {
        OLED_Status("Error", "No firmware");
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    OLED_Status("Flash selected", (char*)headers[ui_fw_index].name);

    int ok = FlashFirmwareFromAT25_Index(ui_fw_index, 1);

    if(ok)
        OLED_Status("Flash OK", "Target started");
    else
        OLED_Status("Flash ERROR", "Check SWD");
}

static void OLED_DoVerifySelected(void)
{
    FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
    uint32_t addresses[MAX_FIRMWARE_COUNT];
    uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

    if(count == 0) {
        OLED_Status("Error", "No firmware");
        return;
    }

    if(ui_fw_index >= count)
        ui_fw_index = count - 1;

    OLED_Status("Verify CRC", "Working...");

    uint32_t crc = 0;
    bool ok = Firmware_Verify(addresses[ui_fw_index], &crc);

    char line[32];
    snprintf(line, sizeof(line), "%08lX", (unsigned long)crc);

    if(ok)
        OLED_Status("CRC OK", line);
    else
        OLED_Status("CRC ERROR", line);
}

static void OLED_DoMemoryInfo(void)
{
    uint32_t total, used, free_space;
    Firmware_GetMemoryInfo(&total, &used, &free_space);

    char line1[32];
    char line2[32];

    snprintf(line1, sizeof(line1), "Used:%lu", (unsigned long)used);
    snprintf(line2, sizeof(line2), "Free:%lu", (unsigned long)free_space);

    OLED_Clear();
    OLED_DrawString(0, 0, "AT25 Memory", 1);
    OLED_DrawString(0, 16, line1, 1);
    OLED_DrawString(0, 26, line2, 1);
    OLED_DrawString(0, 56, "BACK", 1);
    OLED_Update();
}
static void OLED_MenuTask(void)
{
    Key_t key = Buttons_GetKey();

    if(key == KEY_NONE)
        return;

    if(ui_state == UI_MAIN) {
        if(key == KEY_UP) {
            if(ui_menu_index == 0)
                ui_menu_index = 2;
            else
                ui_menu_index--;

            OLED_ShowMainMenu();
        }
        else if(key == KEY_DOWN) {
            ui_menu_index++;
            if(ui_menu_index > 2)
                ui_menu_index = 0;

            OLED_ShowMainMenu();
        }
        else if(key == KEY_OK) {
            if(ui_menu_index == 0) {
                ui_state = UI_SELECT_FW;
                OLED_ShowFirmwareSelect();
            }
            else if(ui_menu_index == 1) {
                ui_state = UI_SELECT_FW;
                OLED_ShowFirmwareSelect();
            }
            else if(ui_menu_index == 2) {
                ui_state = UI_MESSAGE;
                OLED_DoMemoryInfo();
            }
        }
    }
    else if(ui_state == UI_SELECT_FW) {
        FirmwareHeader_t headers[MAX_FIRMWARE_COUNT];
        uint32_t addresses[MAX_FIRMWARE_COUNT];
        uint32_t count = Firmware_GetList(headers, addresses, MAX_FIRMWARE_COUNT);

        if(key == KEY_BACK) {
            ui_state = UI_MAIN;
            OLED_ShowMainMenu();
            return;
        }

        if(count == 0) {
            OLED_ShowFirmwareSelect();
            return;
        }

        if(key == KEY_UP) {
            if(ui_fw_index == 0)
                ui_fw_index = count - 1;
            else
                ui_fw_index--;

            OLED_ShowFirmwareSelect();
        }
        else if(key == KEY_DOWN) {
            ui_fw_index++;
            if(ui_fw_index >= count)
                ui_fw_index = 0;

            OLED_ShowFirmwareSelect();
        }
        else if(key == KEY_OK) {
            ui_state = UI_BUSY;

            if(ui_menu_index == 0)
                OLED_DoFlashSelected();
            else if(ui_menu_index == 1)
                OLED_DoVerifySelected();

            ui_state = UI_MESSAGE;
        }
    }
    else if(ui_state == UI_MESSAGE) {
        if(key == KEY_BACK || key == KEY_OK) {
            ui_state = UI_MAIN;
            OLED_ShowMainMenu();
        }
    }
}


int main(void)
{
    SystemClock_Init();
     I2C_GPIO_Init();
    USART2_Init();

    Buttons_Init();
    OLED_Init();

    OLED_Clear();
    OLED_DrawString(0, 0, "OLED OK", 1);
    OLED_DrawString(0, 16, "Starting...", 1);
    OLED_Update();

    AT25_Init();
    AT25_ForceUnprotect();
    
    UART_Firmware_Init();
    OLED_ShowMainMenu();

    while(1)
    {
        if(USART2->SR & USART_SR_RXNE)
        {
            uint8_t cmd = (uint8_t)USART2->DR;

            if(cmd == CMD_CHECK_TARGET)
                CMD_DirectSwdCheckTarget();
            else if(cmd == CMD_START_PROGRAM)
                CMD_DirectSwdProgramFile();
            else if(cmd == CMD_END_PROGRAM)
                CMD_DirectSwdEndProgram();
            else if(cmd == 'F')
                CMD_FlashFirmwareFromAT25();
            else
                UART_Firmware_ProcessCommand(cmd);
        }
        else
        {
            OLED_MenuTask();
        }
    }
}
