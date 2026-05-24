#define SWD_DRIVER_H
#ifdef SWD_DRIVER_H

#include "stm32f1xx.h"

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

#define FLASH_BASE_ADDR 0x08000000U
#define FLASH_PAGE_SIZE 1024U
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

 void dwt_init(void);



 void swd_gpio_init(void);

 void swd_io_dir_output(void);

 void swd_io_dir_input(void);

 void swd_write_bit(uint8_t bit);

 uint8_t swd_read_bit(void);

 void swd_turnaround(void);

 uint32_t swd_transfer(uint8_t req, uint32_t data);

 void swd_line_reset(void);

 void swd_jtag_to_swd(void);

 uint8_t swd_make_request(uint8_t apndp, uint8_t rnw, uint8_t addr);

 uint32_t swd_read_idcode(void);

 uint32_t swd_read_dp(uint8_t addr);

 void swd_write_dp(uint8_t addr, uint32_t data);

 void swd_write_ap(uint8_t ap, uint8_t addr, uint32_t data);

 uint32_t swd_read_ap(uint8_t ap, uint8_t addr);

 uint32_t swd_mem_read(uint32_t addr);

 void swd_mem_write(uint32_t addr, uint32_t data);

 int swd_init_debug(void);

 void swd_set_csw(uint32_t csw);

 void swd_mem_write16(uint32_t addr, uint16_t data);

uint16_t swd_mem_read16(uint32_t addr);

int flash_wait_ready(void);

void flash_unlock_target(void);

void flash_lock_target(void);

int flash_erase_page(uint32_t page_addr);

int flash_program_halfword(uint32_t addr, uint16_t data);

int flash_program_word32(uint32_t addr, uint32_t data);

int flash_program_block(uint32_t base_addr, const uint32_t *data, uint32_t word_count);

int check_target();

void flash_erase_all_needed(uint32_t base_addr, uint32_t size);


#endif