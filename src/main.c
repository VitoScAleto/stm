#include "../inc/swd.h"
#include <stdint.h>
#include "i2c_at24c256.c"

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

#define CMD_CHECK_TARGET     'C'
#define CMD_START_PROGRAM    'S'
#define CMD_END_PROGRAM      'E'

#define CMD_EEPROM_SAVE      'W'
#define CMD_EEPROM_COUNT     'N'
#define CMD_EEPROM_LIST      'L'
#define CMD_EEPROM_READ      'R'

#define RESP_OK              'K'
#define RESP_ERR             'E'
#define CMD_EEPROM_DELETE    'D'

#define UART_RX_BUF_SIZE     64

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
            // TxStr2((uint8_t *)"PARITY ERROR\r\n");
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

// static const uint32_t flash_image[] =
// {
//     0x20002800,0x08000139,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x00000000,
//     0x00000000,0x00000000,0x00000000,0x08000135,
//     0x08000135,0x00000000,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x080001C1,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x08000135,0x08000135,0x08000135,
//     0x08000135,0x0000E7FE,0x4A0A4B09,0x428B490A,
//     0x2100D306,0x4A0A4B09,0xD3064293,0xB84AF000,
//     0x0B04F852,0x0B04F843,0xF843E7F1,0xE7F31B04,
//     0x20000000,0x08000334,0x20000000,0x20000000,
//     0x20000004,0x881A4B03,0xD5FC0612,0x8098B280,
//     0xBF004770,0x40013800,0x881A4B03,0xD5FC0692,
//     0xB2C08898,0xBF004770,0x40013800,0x2300B538,
//     0x49064605,0x780B700B,0x5CE81C5C,0x700CB2E4,
//     0xFFE0F7FF,0x2B005D2B,0xBD38D1F5,0x20000000,
//     0x4280F04F,0x07DB8A13,0x8A13D50B,0x0301F023,
//     0x0C1B041B,0xF5028213,0x68D33288,0x5300F483,
//     0x477060D3,0xB5082212,0x601A4B3F,0x5380F5A3,
//     0x605A685A,0xF442685A,0x605A6280,0x605A685A,
//     0xF042685A,0x605A0202,0xF442685A,0x605A12E0,
//     0xF442685A,0x605A3280,0xF442681A,0x601A3280,
//     0x0391681A,0x681AD5FC,0x7280F042,0x681A601A,
//     0xD5FC0192,0xF002685A,0x2A08020C,0xF04FD1FA,
//     0x4A2A7100,0x60514C2A,0x680A492A,0x7280F042,
//     0x2100600A,0x60114A28,0x680A4928,0x0201F042,
//     0x2214600A,0x4A26619A,0xF4216851,0x60510170,
//     0xF4416851,0x60511140,0xF442699A,0x619A4280,
//     0x68514A20,0x01F0F021,0x68516051,0x6170F421,
//     0x68516051,0x6192F441,0xF6416051,0xF502514C,
//     0x81115240,0x010CF242,0x69DA8191,0x5180F04F,
//     0x0201F042,0xF04F61DA,0xF6414380,0x851A421F,
//     0x720FF242,0x899A859A,0xF042B292,0x819A0201,
//     0x60114A0D,0x801A2201,0xFF56F7FF,0xD1FB2841,
//     0xF7FF4620,0xE7F7FF5B,0x40022000,0x40010000,
//     0x0800030C,0xE000EDFC,0xE0001004,0xE0001000,
//     0x40011000,0x40010800,0xE000E100,0x6C6C6548,
//     0x57202C6F,0x646C726F,0x000A0D21,0xBF00B5F8,
//     0xBC08BCF8,0x4770469E,0xBF00B5F8,0xBC08BCF8,
//     0x4770469E,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF
// };

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


volatile uint8_t byte = 0;
volatile uint8_t count_byte = 0;
volatile uint32_t word = 0;

typedef enum {
    STATE_WAIT_CMD,
    STATE_WAIT_SIZE,
    STATE_PROGRAM,

    STATE_EEPROM_WAIT_SIZE,
    STATE_EEPROM_RECV_DATA,
    STATE_EEPROM_WAIT_INDEX,
    STATE_EEPROM_WAIT_DELETE_INDEX
} State_Swd;

uint64_t address_flash = 0;
uint32_t size_programm = 0;
uint32_t received_words = 0;

volatile uint8_t byte_ready = 0;
volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
volatile uint32_t rx_count = 0;
uint8_t state_swd = STATE_WAIT_CMD;

/* EEPROM */
uint32_t eeprom_fw_size = 0;
uint32_t eeprom_received = 0;
uint32_t eeprom_write_addr = 0;
uint32_t eeprom_read_index = 0;
uint8_t  eeprom_page_buf[UART_RX_BUF_SIZE];
static void uart_send_u32(uint32_t v)
{
    Tx2((uint8_t)(v & 0xFF));
    Tx2((uint8_t)((v >> 8) & 0xFF));
    Tx2((uint8_t)((v >> 16) & 0xFF));
    Tx2((uint8_t)((v >> 24) & 0xFF));
}

static uint32_t bytes_to_u32(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

static int eeprom_read_u32(uint16_t addr, uint32_t *value)
{
    uint8_t tmp[4];

    if (!AT24C256_ReadBuffer(addr, tmp, 4))
        return 0;

    *value = bytes_to_u32(tmp);
    return 1;
}

static int eeprom_write_u32(uint16_t addr, uint32_t value)
{
    uint8_t tmp[4];

    tmp[0] = (uint8_t)(value & 0xFF);
    tmp[1] = (uint8_t)((value >> 8) & 0xFF);
    tmp[2] = (uint8_t)((value >> 16) & 0xFF);
    tmp[3] = (uint8_t)((value >> 24) & 0xFF);

    return AT24C256_WriteBuffer(addr, tmp, 4);
}

static int eeprom_is_empty_marker(uint32_t value)
{
    return (value == 0xFFFFFFFFu);
}

static int eeprom_find_end(uint16_t *end_addr, uint32_t *count)
{
    uint16_t addr = 0;
    uint32_t cnt = 0;

    while (addr + 4 <= AT24C256_TOTAL_SIZE)
    {
        uint32_t len;

        if (!eeprom_read_u32(addr, &len))
            return 0;

        if (eeprom_is_empty_marker(len))
            break;

        if (len == 0 || (uint32_t)addr + 4u + len > AT24C256_TOTAL_SIZE)
            return 0;

        addr = (uint16_t)(addr + 4u + len);
        cnt++;
    }

    *end_addr = addr;
    *count = cnt;
    return 1;
}

static int eeprom_find_by_index(uint32_t index, uint16_t *data_addr, uint32_t *len_out)
{
    uint16_t addr = 0;
    uint32_t current = 0;

    while (addr + 4 <= AT24C256_TOTAL_SIZE)
    {
        uint32_t len;

        if (!eeprom_read_u32(addr, &len))
            return 0;

        if (eeprom_is_empty_marker(len))
            return 0;

        if (len == 0 || (uint32_t)addr + 4u + len > AT24C256_TOTAL_SIZE)
            return 0;

        if (current == index)
        {
            *data_addr = (uint16_t)(addr + 4u);
            *len_out = len;
            return 1;
        }

        addr = (uint16_t)(addr + 4u + len);
        current++;
    }

    return 0;
}

static void eeprom_send_count(void)
{
    uint16_t end_addr = 0;
    uint32_t count = 0;

    if (!eeprom_find_end(&end_addr, &count))
    {
        uart_send_u32(0);
        return;
    }

    uart_send_u32(count);
}

static void eeprom_send_list(void)
{
    uint16_t addr = 0;
    uint32_t count = 0;
    uint16_t end_addr = 0;

    if (!eeprom_find_end(&end_addr, &count))
    {
        uart_send_u32(0);
        return;
    }

    uart_send_u32(count);

    while (addr < end_addr)
    {
        uint32_t len;
        if (!eeprom_read_u32(addr, &len))
            return;

        uart_send_u32(len);
        addr = (uint16_t)(addr + 4u + len);
    }
}

static int eeprom_begin_save(uint32_t len)
{
    uint16_t end_addr = 0;
    uint32_t count = 0;

    if (len == 0)
        return 0;

    if (!eeprom_find_end(&end_addr, &count))
        return 0;

    if ((uint32_t)end_addr + 4u + len > AT24C256_TOTAL_SIZE)
        return 0;

    if (!eeprom_write_u32(end_addr, len))
        return 0;

    eeprom_write_addr = end_addr + 4u;
    eeprom_fw_size = len;
    eeprom_received = 0;

    return 1;
}

static int eeprom_write_chunk(const uint8_t *data, uint32_t len)
{
    if (eeprom_received + len > eeprom_fw_size)
        return 0;

    if (!AT24C256_WriteBuffer((uint16_t)eeprom_write_addr, data, (uint16_t)len))
        return 0;

    eeprom_write_addr += len;
    eeprom_received += len;
    return 1;
}

static int eeprom_send_firmware(uint32_t index)
{
    uint16_t data_addr;
    uint32_t len;
    uint8_t buf[64];

    if (!eeprom_find_by_index(index, &data_addr, &len))
        return 0;

    uart_send_u32(len);

    uint32_t sent = 0;
    while (sent < len)
    {
        uint16_t chunk = (len - sent > sizeof(buf)) ? sizeof(buf) : (uint16_t)(len - sent);

        if (!AT24C256_ReadBuffer((uint16_t)(data_addr + sent), buf, chunk))
            return 0;

        for (uint16_t i = 0; i < chunk; i++)
            Tx2(buf[i]);

        sent += chunk;
    }

    return 1;
}
static int eeprom_find_record_by_index(uint32_t index, uint16_t *record_addr, uint32_t *record_len)
{
    uint16_t addr = 0;
    uint32_t current = 0;

    while (addr + 4 <= AT24C256_TOTAL_SIZE)
    {
        uint32_t len;

        if (!eeprom_read_u32(addr, &len))
            return 0;

        if (eeprom_is_empty_marker(len))
            return 0;

        if (len == 0 || (uint32_t)addr + 4u + len > AT24C256_TOTAL_SIZE)
            return 0;

        if (current == index)
        {
            *record_addr = addr;
            *record_len = len;
            return 1;
        }

        addr = (uint16_t)(addr + 4u + len);
        current++;
    }

    return 0;
}
static int eeprom_delete_firmware(uint32_t index)
{
    uint16_t record_addr = 0;
    uint32_t record_len = 0;

    uint16_t end_addr = 0;
    uint32_t count = 0;

    uint16_t delete_total_size;
    uint16_t src_addr;
    uint16_t dst_addr;

    uint8_t buf[64];
    uint8_t ff[64];

    for (uint16_t i = 0; i < sizeof(ff); i++)
        ff[i] = 0xFF;

    /* найти запись по индексу */
    if (!eeprom_find_record_by_index(index, &record_addr, &record_len))
        return 0;

    /* найти конец всех записей */
    if (!eeprom_find_end(&end_addr, &count))
        return 0;

    delete_total_size = (uint16_t)(4u + record_len);

    /*
        src_addr = начало следующей записи после удаляемой
        dst_addr = куда сдвигать
    */
    src_addr = (uint16_t)(record_addr + delete_total_size);
    dst_addr = record_addr;

    /*
        Сдвигаем всё, что справа от удаляемой записи, влево.
        Работаем блоками по 64 байта.
    */
    while (src_addr < end_addr)
    {
        uint16_t remain = (uint16_t)(end_addr - src_addr);
        uint16_t chunk = (remain > sizeof(buf)) ? sizeof(buf) : remain;

        if (!AT24C256_ReadBuffer(src_addr, buf, chunk))
            return 0;

        if (!AT24C256_WriteBuffer(dst_addr, buf, chunk))
            return 0;

        src_addr = (uint16_t)(src_addr + chunk);
        dst_addr = (uint16_t)(dst_addr + chunk);
    }

    uint16_t tail_start = (uint16_t)(end_addr - delete_total_size);
    uint16_t tail_remain = delete_total_size;

    while (tail_remain > 0)
    {
        uint16_t chunk = (tail_remain > sizeof(ff)) ? sizeof(ff) : tail_remain;

        if (!AT24C256_WriteBuffer(tail_start, ff, chunk))
            return 0;

        tail_start = (uint16_t)(tail_start + chunk);
        tail_remain = (uint16_t)(tail_remain - chunk);
    }

    return 1;
}
void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        uint8_t b = (uint8_t)USART2->DR;

        if (state_swd == STATE_PROGRAM || state_swd == STATE_WAIT_SIZE || state_swd == STATE_EEPROM_WAIT_SIZE || state_swd == STATE_EEPROM_WAIT_INDEX || state_swd == STATE_EEPROM_WAIT_DELETE_INDEX)
        {
            word |= ((uint32_t)b << (count_byte * 8));
            count_byte++;
        }
        else if (state_swd == STATE_EEPROM_RECV_DATA)
        {
            if (rx_count < UART_RX_BUF_SIZE)
            {
                rx_buf[rx_count++] = b;
            }
        }
        else
        {
            byte = b;
            byte_ready = 1;
        }
    }
}
int main(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
    }

    FLASH->ACR = FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    SystemCoreClock = 72000000;

    swd_gpio_init();
    dwt_init();

    USART2_Init();
    I2C1_Init_AT24C256();
    
    
    while (1)
    {
        switch (state_swd)
        {
            case STATE_WAIT_CMD:
            {
                if (byte_ready)
                {
                    uint8_t cmd = byte;
                    byte_ready = 0;
                    byte = 0;

                    if (cmd == CMD_CHECK_TARGET)
                    {
                        if (swd_init_debug())
                            Tx2(RESP_OK);
                        else
                            Tx2(RESP_ERR);
                    }
                    else if (cmd == CMD_START_PROGRAM)
                    {
                        count_byte = 0;
                        word = 0;
                        state_swd = STATE_WAIT_SIZE;
                    }
                    else if (cmd == CMD_END_PROGRAM)
                    {
                        swd_mem_write(0xE000ED0C, 0x05FA0004);
                        Tx2(RESP_OK);
                    }
                    else if (cmd == CMD_EEPROM_SAVE)
                    {
                        count_byte = 0;
                        word = 0;
                        state_swd = STATE_EEPROM_WAIT_SIZE;
                    }
                    else if (cmd == CMD_EEPROM_COUNT)
                    {
                        eeprom_send_count();
                    }
                    else if (cmd == CMD_EEPROM_LIST)
                    {
                        eeprom_send_list();
                    }
                    else if (cmd == CMD_EEPROM_READ)
                    {
                        count_byte = 0;
                        word = 0;
                        state_swd = STATE_EEPROM_WAIT_INDEX;
                    }
                    else if (cmd == CMD_EEPROM_DELETE)
                    {
                        count_byte = 0;
                        word = 0;
                        state_swd = STATE_EEPROM_WAIT_DELETE_INDEX;
                    }
                }
            } break;

            case STATE_WAIT_SIZE:
            {
                if (count_byte == 4)
                {
                    size_programm = word;
                    received_words = 0;
                    address_flash = 0;

                    word = 0;
                    count_byte = 0;

                    flash_unlock_target();
                    Tx2(RESP_OK);

                    state_swd = STATE_PROGRAM;
                }
            } break;

            case STATE_PROGRAM:
            {
                if (count_byte == 4)
                {
                    if ((address_flash * 4u) % 0x400u == 0u)
                    {
                        if (!flash_erase_page(FLASH_BASE + address_flash * 4u))
                        {
                            flash_lock_target();
                            Tx2(RESP_ERR);
                            state_swd = STATE_WAIT_CMD;
                            break;
                        }
                    }

                    if (!flash_program_word32(FLASH_BASE + address_flash * 4u, word))
                    {
                        flash_lock_target();
                        Tx2(RESP_ERR);
                        state_swd = STATE_WAIT_CMD;
                        break;
                    }

                    for (int i = 0; i < 4; i++)
                    {
                        Tx2((word >> (i * 8)) & 0xFF);
                    }

                    address_flash++;
                    received_words++;

                    word = 0;
                    count_byte = 0;

                    if (received_words >= size_programm)
                    {
                        flash_lock_target();
                        state_swd = STATE_WAIT_CMD;
                    }
                }
            } break;

            case STATE_EEPROM_WAIT_SIZE:
            {
                if (count_byte == 4)
                {
                    uint32_t len = word;

                    word = 0;
                    count_byte = 0;
                    rx_count = 0;

                    if (eeprom_begin_save(len))
                    {
                        Tx2(RESP_OK);
                        state_swd = STATE_EEPROM_RECV_DATA;
                    }
                    else
                    {
                        Tx2(RESP_ERR);
                        state_swd = STATE_WAIT_CMD;
                    }
                }
            } break;

            case STATE_EEPROM_RECV_DATA:
            {
                if (rx_count > 0)
                {
                    uint32_t remain = eeprom_fw_size - eeprom_received;
                    uint32_t chunk = rx_count;

                    if (chunk > remain)
                        chunk = remain;

                    for (uint32_t i = 0; i < chunk; i++)
                        eeprom_page_buf[i] = rx_buf[i];

                    rx_count = 0;

                    if (!eeprom_write_chunk(eeprom_page_buf, chunk))
                    {
                        Tx2(RESP_ERR);
                        state_swd = STATE_WAIT_CMD;
                        break;
                    }

                    Tx2(RESP_OK);

                    if (eeprom_received >= eeprom_fw_size)
                    {
                        Tx2(RESP_OK);
                        state_swd = STATE_WAIT_CMD;
                    }
                }
            } break;

            case STATE_EEPROM_WAIT_INDEX:
            {
                if (count_byte == 4)
                {
                    uint32_t index = word;

                    word = 0;
                    count_byte = 0;

                    if (!eeprom_send_firmware(index))
                    {
                        uart_send_u32(0);
                    }

                    state_swd = STATE_WAIT_CMD;
                }
            } break;
            case STATE_EEPROM_WAIT_DELETE_INDEX:
            {
                if (count_byte == 4)
                {
                    uint32_t index = word;

                    word = 0;
                    count_byte = 0;

                    if (eeprom_delete_firmware(index))
                        Tx2(RESP_OK);
                    else
                        Tx2(RESP_ERR);

                    state_swd = STATE_WAIT_CMD;
                }
            } break;
        }
    }
    }

