#include "../inc/swd.h"
#include <stdint.h>
#include <string.h>

// ================= КОНФИГУРАЦИЯ =================
// Пины мастера
#define SWCLK_PORT   GPIOA
#define SWCLK_PIN    8
#define SWDIO_PORT   GPIOA
#define SWDIO_PIN    9
#define NRST_PORT    GPIOB
#define NRST_PIN     0
#define LED_PORT     GPIOC
#define LED_PIN      13

// Адреса регистров отладки
#define DP_RDBUFF 0x0C
#define DP_ABORT 0x00
#define DP_IDCODE    0x00
#define DP_CTRLSTAT  0x04
#define DP_SELECT    0x08
#define AP_CSW       0x00 // Control/Status Word (настройка доступа)
#define AP_TAR       0x04 // Transfer Address (адрес в целевой системе)
#define AP_DRW       0x0C // Data Read/Write (данные)
#define AP_IDR 0xFC

// Флаги DP_CTRLSTAT
#define CSYSPWRUPACK (1 << 31) // Подтверждение включения питания системного домена  
#define CDBGPWRUPACK (1 << 29) // Подтверждение включения отладки
#define CSYSPWRUPREQ (1 << 30) // Запрос на включение системы
#define CDBGPWRUPREQ (1 << 28) // Запрос на включение отладки

// Адрес Flash контроллера STM32F1
#define FLASH_ACR    0x40022000
#define FLASH_KEYR   0x40022004
#define FLASH_CR     0x40022010
#define FLASH_SR     0x4002200C

// Ключи разблокировки Flash
#define KEY1         0x45670123
#define KEY2         0xCDEF89AB

#define SWDIO_HIGH() GPIOA->BSRR = (1 << SWDIO_PIN)
#define SWDIO_LOW() GPIOA->BSRR = (1 << (SWDIO_PIN+16))
#define SWCLK_HIGH() GPIOA->BSRR = (1 << SWCLK_PIN)
#define SWCLK_LOW() GPIOA->BSRR = (1 << (SWCLK_PIN+16))
#define NRST_HIGH() GPIOB->BSRR = (1 << NRST_PIN)
#define NRST_LOW() GPIOB->BSRR = (1 << (16+NRST_PIN))


// static __INLINE void delay_us(uint32_t us) {
//     SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;
//     SysTick->VAL = 0;                 // Сбрасываем текущий счётчик
//     SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk; // источник = ядро

//     // Ждем переполнения
//     while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

//     SysTick->CTRL = 0; 
// }

// static __INLINE void delay_ns(uint32_t ns) {
//     uint32_t ticks = (ns * SystemCoreClock) / 1000000000;
//     TIM2->CNT = 0;
//     TIM2->CR1 |= TIM_CR1_CEN; // включаем таймер
//     while(TIM2->CNT < ticks);
//     TIM2->CR1 &= ~TIM_CR1_CEN;
// }

// static __INLINE void delay_ms(uint32_t ms) {
//     while(ms--) delay_us(999);
// }

// static inline void swd_half_delay(void)
// {
//     /* При 72 МГц, для SWD 4 МГц: 
//      * Полутакта = 125 нс ≈ 9 циклов
//      * Вычитаем накладные расходы (~4 цикла) */
//     __asm volatile(
//         "nop\n" "nop\n" "nop\n" "nop\n"
//         ::: "memory"
//     );
// }

// static inline void swd_clock_cycle(void)
// {
//     SWCLK_LOW();
//     __asm volatile(
//         "nop\n" "nop\n" "nop\n" "nop\n"
//         ::: "memory"
//     );
//     SWCLK_HIGH();
//     __asm volatile(
//         "nop\n" "nop\n" "nop\n" "nop\n" 
//         ::: "memory"
//     );
// }

static void dwt_init(void)
{
    // Разрешить доступ к DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Сбросить счётчик
    DWT->CYCCNT = 0;

    // Включить счётчик тактов
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles);
}

static void led_on(void) {
    LED_PORT->BSRR = (1 << LED_PIN);
}

static void led_off(void) {
    LED_PORT->BRR = (1 << LED_PIN);
}

static void led_toggle(void) {
    LED_PORT->ODR ^= (1 << LED_PIN);
}

//  низкоуровневый SWD 
static void swd_gpio_init(void) {
    // Включение тактирования портов
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    
    // Настройка SWCLK (PA8) как выход Push-Pull 50MHz
    SWCLK_PORT->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    SWCLK_PORT->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0; // 50 MHz
    
    // Настройка SWDIO (PA9) - сначала как выход
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    SWDIO_PORT->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0; // 50 MHz
    
    // Настройка NRST (PB0) как выход
    NRST_PORT->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    NRST_PORT->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0;
    
    // Настройка LED (PC13) как выход
    LED_PORT->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    LED_PORT->CRH |= GPIO_CRH_MODE13_1;
    
    // Начальные состояния
    SWCLK_LOW();  
    NRST_PORT->BSRR = (1 << NRST_PIN);    // NRST = 1
    LED_PORT->BRR = (1 << LED_PIN);       // LED off
}

static void swd_io_dir_output(void) {
    // SWDIO как выход 
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9);
    SWDIO_PORT->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0; // 50 MHz
}

static void swd_io_dir_input(void)
{
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    SWDIO_PORT->CRH |= GPIO_CRH_CNF9_1;      // input pull-up/pull-down
    SWDIO_PORT->ODR |= (1 << SWDIO_PIN);     // pull-up
}


static void swd_write_bit(uint8_t bit)
{
    if(bit) SWDIO_HIGH();
    else SWDIO_LOW();

    // Тактирование
    SWCLK_LOW();
    delay_us(1);

    SWCLK_HIGH();
     delay_us(1);
}

// static uint8_t swd_read_bit(void) не правильно
// {
//     uint8_t bit;
//     // SWCLK LOW
//     SWCLK_LOW();
//     delay_us(1);

//     // SWCLK HIGH — на фронте данные стабильны, читаем здесь
//     SWCLK_HIGH();
//      delay_us(1);

//     bit = (SWDIO_PORT->IDR >> SWDIO_PIN) & 1;

//     return bit;
// }

static uint8_t swd_read_bit(void)//правильно
{
    uint8_t bit;
    
    // SWCLK LOW - таргет выставляет данные на линии
    SWCLK_LOW();
    delay_us(1);
    
    // Читаем бит, пока SWCLK LOW (данные стабильны)
    bit = (SWDIO_PORT->IDR >> SWDIO_PIN) & 1;
    
    // SWCLK HIGH - строббируем (формируем фронт для следующего бита)
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


// -------------------------
// Основная функция передачи SWD
// -------------------------
static uint32_t swd_transfer(uint8_t req, uint32_t data)
{
    uint32_t val = 0;
    uint8_t ack;

    // 1. Отправка запроса
    swd_io_dir_output();
    for(int i = 0; i < 8; i++)
        swd_write_bit((req >> i) & 1);

   
    // 2. Turnaround к target
    swd_io_dir_input();
    swd_turnaround();
    

    // 3. Читаем ACK
    ack = 0;
    for(int i = 0; i < 3; i++)
        ack |= (swd_read_bit() << i);


  if(req & (1<<2)) {
    val = 0;
    uint8_t parity_calc = 0;
    for(int i=0;i<32;i++) {
        uint8_t b = swd_read_bit();
        val |= ((uint32_t)b << i);
        parity_calc ^= b;
    }

    uint8_t parity_bit = swd_read_bit();
    if(parity_calc != parity_bit)
        TxStr2((uint8_t*)"PARITY ERROR\r\n");

    // 5. Turnaround обратно к мастеру
    swd_io_dir_output();
    swd_turnaround();
}
    else // WRITE
    {
        // turnaround к output
        swd_turnaround();
        swd_io_dir_output();

        for(int i = 0; i < 32; i++)
            swd_write_bit((data >> i) & 1);

        uint8_t parity = __builtin_popcount(data) & 1;
        swd_write_bit(parity);

        // 1 idle цикл
        swd_write_bit(0);
    }

    return val;
}


static void swd_line_reset(void) {
    // 1. Настраиваем SWDIO как выход
    swd_io_dir_output();

    // 2. Устанавливаем SWDIO = 1
    SWDIO_HIGH();

    // 3. Генерируем 55+ импульсов SWCLK при SWDIO=1
    for(int i = 0; i < 55; i++) {
        SWCLK_LOW();
        delay_us(1);

        SWCLK_HIGH();
         delay_us(1);

    }
}

static void swd_jtag_to_swd(void) {
    // Специальная последовательность для переключения JTAG->SWD
    uint16_t switch_sequence = 0xE79E; // 0b1110011110011110
    //мб 0x79E7
    
    for (int i = 0; i < 16; i++) {
        if (switch_sequence & 0x01) {
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

// ================= ВЫСОКОУРОВНЕВЫЕ ФУНКЦИИ =================
static uint8_t swd_make_request(uint8_t apndp, uint8_t rnw, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 1;
    uint8_t a3 = (addr >> 3) & 1;

    uint8_t parity = (apndp ^ rnw ^ a2 ^ a3) & 1;

    uint8_t req = 0;
    req |= (1 << 0);            // Start 1
    req |= (apndp << 1);        // APnDP 
    req |= (rnw << 2);          // RnW 
    req |= (a2 << 3);           // A2 
    req |= (a3 << 4);           // A3 
    req |= (parity << 5);       // Parity 
    req |= (0 << 6);            // Stop (0)
    req |= (1 << 7);            // Park (1)

    return req;
}

static uint32_t swd_read_idcode(void) {
    // Чтение IDCODE (DPACC, READ, адрес 0)
    uint8_t req = (1 << 0) |          // START
                  (0 << 1) |          // APnDP=0 (DP)
                  (1 << 2) |          // RnW=1 (Read)
                  (0 << 3) |          // A2=0
                  (0 << 4) |            // A3=0
                  (1 << 5) |            // parity 1
                  (0<< 6)   |               // stop 0
                  (1 << 7);      // park
    
    return swd_transfer(req, 0);
}

static uint32_t swd_read_dp(uint8_t addr)
{
    uint8_t req = swd_make_request(0, 1, addr); // DP, Read
    return swd_transfer(req, 0);
}

static void swd_write_dp(uint8_t addr, uint32_t data)
{
    uint8_t req = swd_make_request(0, 0, addr); // DP, Write
    swd_transfer(req, data);
}
static void swd_clear_errors(void)
{
    // STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR
    swd_write_dp(DP_ABORT, 0x1E);
}



static void swd_write_ap(uint8_t ap, uint8_t addr, uint32_t data)
{
    swd_write_dp(DP_SELECT, (ap << 24) | (addr & 0xF0));

    uint8_t req = swd_make_request(1, 0, addr); // AP, Write
    swd_transfer(req, data);
}

static uint32_t swd_read_ap(uint8_t ap, uint8_t addr)
{
    // Выбор AP и банка
   swd_write_dp(DP_SELECT, (ap << 24) | (addr & 0xF0));

    // 1️⃣ Запускаем AP read (данные ещё не здесь!)
    uint8_t req = swd_make_request(1, 1, addr);
    swd_transfer(req, 0);

    // 2️⃣ Читаем результат из DP_RDBUFF
    req = swd_make_request(0, 1, DP_RDBUFF);
    return swd_transfer(req, 0);
}

static uint32_t swd_mem_read(uint32_t addr) {
    // Записываем адрес
    swd_write_ap(0, AP_TAR, addr);
    // Читаем данные
    return swd_read_ap(0, AP_DRW);
}


static int swd_init_debug(void) {
    // Инициализация отладочного порта
    
    swd_line_reset();
    
    swd_jtag_to_swd();
    swd_line_reset();

    SWDIO_LOW();
    swd_turnaround();
    swd_turnaround();

    // Читаем IDCODE
    uint32_t id = swd_read_idcode();
    if(id == 0xFFFFFFFF || id == 0) {
        return 0; // Ошибка
    }
    
    // Включаем питание отладки
   swd_write_dp(DP_CTRLSTAT, CSYSPWRUPREQ | CDBGPWRUPREQ);
   
//     uint32_t stat;
//  do {
//     stat = swd_read_dp(DP_CTRLSTAT);
// } while((stat & (CSYSPWRUPACK | CDBGPWRUPACK)) !=
//         (CSYSPWRUPACK | CDBGPWRUPACK));
    
    // Выбираем AP #0
   swd_write_dp(DP_SELECT, 0x00000000);
    
    // // Настраиваем AP (32-bit access, auto-increment)
   swd_write_ap(0, AP_CSW, 0x22000012);
    
 
    return 1; // Успех
}

static void swd_mem_write(uint32_t addr, uint32_t data) {
    // Записываем адрес
    swd_write_ap(0, AP_TAR, addr);
    // Записываем данные
    swd_write_ap(0, AP_DRW, data);
}



// ================= Flash операции =================
static void flash_unlock() {
    // Разблокировка Flash контроллера
    swd_mem_write(FLASH->KEYR, KEY1);
    swd_mem_write(FLASH->KEYR, KEY2);
}

static void flash_erase_all(uint32_t flash_base) {
    // Стирание всей Flash
    swd_mem_write(flash_base + FLASH_CR, 0x00000002); // MER=1
    swd_mem_write(flash_base + FLASH_CR, 0x00000042); // MER=1, STR=1
    
    // Ждем завершения
    while(swd_mem_read(flash_base + FLASH_SR) & 0x00000001) {
        // BSY=1 - ждем
    }
    
    swd_mem_write(flash_base + FLASH_CR, 0x00000000); // Сброс
}

static void flash_program_word(uint32_t addr, uint32_t data, uint32_t flash_base) {
    // Программирование одного слова
    swd_mem_write(flash_base + FLASH_CR, 0x00000001); // PG=1
    swd_mem_write(addr, data);
    
   // Ждем завершения
    while(swd_mem_read(flash_base + FLASH_SR) & 0x00000001) {
        // BSY=1 - ждем
    }
    
    swd_mem_write(flash_base + FLASH_CR, 0x00000000); // Сброс
}
#define FLASH_KEYR_ADDR     0x40022004u
#define FLASH_SR_ADDR       0x4002200Cu
#define FLASH_CR_ADDR       0x40022010u
#define FLASH_AR_ADDR       0x40022014u

#define FLASH_SR_BSY        (1u << 0)
#define FLASH_SR_PGERR      (1u << 2)
#define FLASH_SR_WRPRTERR   (1u << 4)
#define FLASH_SR_EOP        (1u << 5)

#define FLASH_CR_PG         (1u << 0)
#define FLASH_CR_PER        (1u << 1)
#define FLASH_CR_MER        (1u << 2)
#define FLASH_CR_STRT       (1u << 6)
#define FLASH_CR_LOCK       (1u << 7)

#define FLASH_KEY1_VALUE    0x45670123u
#define FLASH_KEY2_VALUE    0xCDEF89ABu


#define AP_CSW_32BIT  0x22000012u
#define AP_CSW_16BIT  0x22000011u

#define FLASH_PAGE0_ADDR 0x08000000u

static void swd_set_csw(uint32_t csw)
{
    swd_write_ap(0, AP_CSW, csw);
}

static void swd_mem_write16(uint32_t addr, uint16_t data)
{
    uint32_t payload;

    swd_write_ap(0, AP_CSW, AP_CSW_16BIT);
    swd_write_ap(0, AP_TAR, addr);

    // Для halfword доступов через DRW:
    // addr[1] = 0 -> данные в bits[15:0]
    // addr[1] = 1 -> данные в bits[31:16]
    if (addr & 0x2u)
        payload = ((uint32_t)data) << 16;
    else
        payload = (uint32_t)data;

    swd_write_ap(0, AP_DRW, payload);

    swd_write_ap(0, AP_CSW, AP_CSW_32BIT);
}

static uint16_t swd_mem_read16(uint32_t addr)
{
    uint32_t v;

    swd_write_ap(0, AP_CSW, AP_CSW_16BIT);
    swd_write_ap(0, AP_TAR, addr);
    v = swd_read_ap(0, AP_DRW);
    swd_write_ap(0, AP_CSW, AP_CSW_32BIT);

    if (addr & 0x2u)
        return (uint16_t)(v >> 16);
    else
        return (uint16_t)(v & 0xFFFFu);
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
    if (!flash_wait_ready()) return 0;

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
    if (addr & 1) return 0;

    if (!flash_wait_ready()) return 0;

    // Включаем режим программирования
    swd_mem_write(FLASH_CR_ADDR, FLASH_CR_PG);

    // Записываем 16 бит
    swd_mem_write16(addr, data);

    // Ждём окончания
    if (!flash_wait_ready()) {
        swd_mem_write(FLASH_CR_ADDR, 0);
        return 0;
    }

    // Выключаем PG
    swd_mem_write(FLASH_CR_ADDR, 0);

    return 1;
}

static int flash_program_word32(uint32_t addr, uint32_t data)
{
    if (addr & 3u) return 0;

    if (!flash_program_halfword(addr,     (uint16_t)(data & 0xFFFFu))) return 0;
    if (!flash_program_halfword(addr + 2, (uint16_t)(data >> 16)))     return 0;

    return 1;
}

static const uint32_t flash_image[] =
{
0x20002800,0x08000139,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x00000000,
0x00000000,0x00000000,0x00000000,0x08000135,
0x08000135,0x00000000,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x080001C1,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x08000135,0x08000135,0x08000135,
0x08000135,0x0000E7FE,0x4A0A4B09,0x428B490A,
0x2100D306,0x4A0A4B09,0xD3064293,0xB84AF000,
0x0B04F852,0x0B04F843,0xF843E7F1,0xE7F31B04,
0x20000000,0x08000334,0x20000000,0x20000000,
0x20000004,0x881A4B03,0xD5FC0612,0x8098B280,
0xBF004770,0x40013800,0x881A4B03,0xD5FC0692,
0xB2C08898,0xBF004770,0x40013800,0x2300B538,
0x49064605,0x780B700B,0x5CE81C5C,0x700CB2E4,
0xFFE0F7FF,0x2B005D2B,0xBD38D1F5,0x20000000,
0x4280F04F,0x07DB8A13,0x8A13D50B,0x0301F023,
0x0C1B041B,0xF5028213,0x68D33288,0x5300F483,
0x477060D3,0xB5082212,0x601A4B3F,0x5380F5A3,
0x605A685A,0xF442685A,0x605A6280,0x605A685A,
0xF042685A,0x605A0202,0xF442685A,0x605A12E0,
0xF442685A,0x605A3280,0xF442681A,0x601A3280,
0x0391681A,0x681AD5FC,0x7280F042,0x681A601A,
0xD5FC0192,0xF002685A,0x2A08020C,0xF04FD1FA,
0x4A2A7100,0x60514C2A,0x680A492A,0x7280F042,
0x2100600A,0x60114A28,0x680A4928,0x0201F042,
0x2214600A,0x4A26619A,0xF4216851,0x60510170,
0xF4416851,0x60511140,0xF442699A,0x619A4280,
0x68514A20,0x01F0F021,0x68516051,0x6170F421,
0x68516051,0x6192F441,0xF6416051,0xF502514C,
0x81115240,0x010CF242,0x69DA8191,0x5180F04F,
0x0201F042,0xF04F61DA,0xF6414380,0x851A421F,
0x720FF242,0x899A859A,0xF042B292,0x819A0201,
0x60114A0D,0x801A2201,0xFF56F7FF,0xD1FB2841,
0xF7FF4620,0xE7F7FF5B,0x40022000,0x40010000,
0x0800030C,0xE000EDFC,0xE0001004,0xE0001000,
0x40011000,0x40010800,0xE000E100,0x6C6C6548,
0x57202C6F,0x646C726F,0x000A0D21,0xBF00B5F8,
0xBC08BCF8,0x4770469E,0xBF00B5F8,0xBC08BCF8,
0x4770469E,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF
};

static int flash_program_block(uint32_t base_addr,
                               const uint32_t *data,
                               uint32_t word_count)
{
    // flash_erase_page(base_addr);

    for (uint32_t i = 0; i < word_count; i++) {
        if(i * 4u  % 0x400 == 0) flash_erase_page(base_addr +i * 4u );
        if (!flash_program_word32(base_addr + i * 4u, data[i])) {
            return 0;
        }
    }
    return 1;
}

int main(void) {
    // ------------------ Тактирование 72 МГц ------------------
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR = FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = 72000000;

    // ------------------ GPIO + SWD ------------------
    swd_gpio_init();
    dwt_init();

    // ------------------ USART2 ------------------
     USART2_Init();

     TxStr2((uint8_t*)"USART OK\r\n");

    // ------------------ SWD init ------------------
    if (!swd_init_debug()) {
        TxStr2((uint8_t*)"SWD FAIL\r\n");

        while (1) {
            led_toggle();
            delay_us(300000);
        }
    }

    // uint32_t addr = 0x4001100C;
    // uint32_t data = swd_mem_read(addr);
    // TxStr2((uint8_t*)"0x");
    // TxHex32(addr);
    // TxStr2((uint8_t*)": 0x");
    // TxHex32(data);
    // TxStr2((uint8_t*)"\r\n");
    // data ^= (1 << 13);
    // swd_mem_write(0x4001100C, data);
    // delay_us(10);
    // data = swd_mem_read(addr);
    // TxStr2((uint8_t*)"0x");
    // TxHex32(addr);
    // TxStr2((uint8_t*)": 0x");
    // TxHex32(data);
    // TxStr2((uint8_t*)"\r\n");
    // delay_us(1000);

    // flash_unlock();
    // flash_erase_all(FLASH_BASE);

    flash_unlock_target();


    flash_program_block(0x08000000,
                        flash_image,
                        sizeof(flash_image)/4);
    
    
    flash_lock_target();
    // uint32_t start_addr = 0x4001100C;
    // uint32_t num_words = 100;  // количество 32-битных слов для чтения

    // TxStr2((uint8_t*)"\r\nMemory Dump:\r\n");
    // for(uint32_t i = 0; i < num_words; i++) 
    // {
    //     uint32_t addr = start_addr + (i * 4);
    //     uint32_t data = swd_mem_read(addr);
        
    //     // Вывод в формате: адрес : значение
    //     TxStr2((uint8_t*)"0x");
    //     TxHex32(addr);
    //     TxStr2((uint8_t*)": 0x");
    //     TxHex32(data);
    //     TxStr2((uint8_t*)"\r\n");
    // }
}