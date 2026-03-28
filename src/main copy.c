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
    if (addr & 1u) return 0;
    if (!flash_wait_ready()) return 0;

    swd_mem_write(FLASH_CR_ADDR, FLASH_CR_PG);
    swd_mem_write(addr, data);

    if (!flash_wait_ready()) {
        swd_mem_write(FLASH_CR_ADDR, 0);
        return 0;
    }

    swd_mem_write(FLASH_CR_ADDR, 0);

    return (swd_mem_read(addr) == data);
}
static int flash_program_word32(uint32_t addr, uint32_t data)
{
    if (addr & 3u) return 0;

    if (!flash_program_halfword(addr, (uint16_t)(data & 0xFFFFu))) return 0;
    if (!flash_program_halfword(addr + 2u, (uint16_t)(data >> 16))) return 0;

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
// flash_unlock_target();
// TxStr2("Erase\r\n");
// if (!flash_erase_page(0x08000000u)) {
//     TxStr2((uint8_t*)"ERASE FAIL\r\n");
// }

// flash_program_word(0x08000004u, 0x12345678u,FLASH_BASE); 
// if (!flash_program_word32(0x08000004u, 0x12345678u)) {
//     TxStr2((uint8_t*)"PROGRAM FAIL\r\n");
// }

// flash_lock_target();

    uint32_t start_addr = 0x4001100C;
uint32_t num_words = 100;  // количество 32-битных слов для чтения

TxStr2((uint8_t*)"\r\nMemory Dump:\r\n");
for(uint32_t i = 0; i < num_words; i++) 
{
    uint32_t addr = start_addr + (i * 4);
    uint32_t data = swd_mem_read(addr);
    
    // Вывод в формате: адрес : значение
    TxStr2((uint8_t*)"0x");
    TxHex32(addr);
    TxStr2((uint8_t*)": 0x");
    TxHex32(data);
    TxStr2((uint8_t*)"\r\n");
}
}