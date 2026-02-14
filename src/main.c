#include "../inc/stm32f1xx.h"
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
#define DP_IDCODE    0x00
#define DP_CTRLSTAT  0x04
#define DP_SELECT    0x08
#define AP_CSW       0x00 // Control/Status Word (настройка доступа)
#define AP_TAR       0x04 // Transfer Address (адрес в целевой системе)
#define AP_DRW       0x0C // Data Read/Write (данные)

// Флаги DP_CTRLSTAT
#define CSYSPWRUPACK (1 << 31) // Подтверждение включения системы  
#define CDBGPWRUPACK (1 << 27) // Подтверждение включения отладки
#define CSYSPWRUPREQ (1 << 30) // Запрос на включение системы
#define CDBGPWRUPREQ (1 << 26) // Запрос на включение отладки

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


static __INLINE void delay_us(uint32_t us) {
    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;
    SysTick->VAL = 0;                 // Сбрасываем текущий счётчик
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk; // источник = ядро

    // Ждем переполнения
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

    SysTick->CTRL = 0; 
}

static __INLINE void delay_ns(uint32_t ns) {
    uint32_t ticks = (ns * SystemCoreClock) / 1000000000;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN; // включаем таймер
    while(TIM2->CNT < ticks);
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

static __INLINE void delay_ms(uint32_t ms) {
    while(ms--) delay_us(999);
}

static inline void swd_half_delay(void)
{
    /* При 72 МГц, для SWD 4 МГц: 
     * Полутакта = 125 нс ≈ 9 циклов
     * Вычитаем накладные расходы (~4 цикла) */
    __asm volatile(
        "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
        ::: "memory"
    );
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

static void swd_io_dir_input(void) {
    // SWDIO как вход
    SWDIO_PORT->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    SWDIO_PORT->CRH |= GPIO_CRH_CNF9_0; // Floating input
}

static void swd_write_bit(uint8_t bit) {
    if(bit == 1) {
       SWDIO_HIGH();  // HIGH
    } else {
       SWDIO_LOW();   // LOW
    }
    
    SWCLK_LOW();  // CLK = 0
    delay_us(2);
    SWCLK_HIGH(); // CLK = 1
    delay_us(2);
}

static uint8_t swd_read_bit(void)
{
    uint8_t bit;

    SWCLK_LOW();
    delay_us(2);

    SWCLK_HIGH();                 
    delay_us(2);

    bit = (SWDIO_PORT->IDR >> SWDIO_PIN) & 1;  

    return bit;
}

static void swd_turnaround(void)
{
    SWCLK_LOW();
    delay_us(2);
    SWCLK_HIGH();                 
    delay_us(2);
}

// ================= SWD протокол =================
static uint32_t swd_transfer(uint8_t req, uint32_t data) {//FIXME: Пересмотреть
    uint8_t ack;
    uint32_t val = 0;
    uint8_t parity;
    
    //Формируем запрос с парностью
    parity = __builtin_popcount(req & 0x1F) & 1;
    req = (req & 0xDF) | (parity << 5);

    //FIXME: missing park bit

    // 2. Отправляем 8 бит запроса
    swd_io_dir_output();
    for(int i = 0; i < 8; i++) {
        swd_write_bit((req >> i) & 1);
    }
    swd_io_dir_input();

    
    // 3. Turnaround цикл
    swd_turnaround();
    

    // 4. Читаем ACK (3 бита)
    ack = 0;
    for(int i = 0; i < 3; i++) {
        ack |= (swd_read_bit() << i);
    }
    
    if(ack != 1) { // 001 = OK
        return 0xFFFFFFFF; // Ошибка
    }
   

    // 5. Передача данных
    if(req & (1 << 2)) { // Чтение (RnW=1)
        // Читаем 32 бита данных
        for(int i = 0; i < 32; i++) {
            if(swd_read_bit()) {
                val |= (1 << i);
            }
        }
        // Читаем парность (пропускаем)
        swd_read_bit();
    } else { // Запись (RnW=0)

        swd_turnaround();
        // Пишем 32 бита данных
        swd_io_dir_output();

        for(int i = 0; i < 32; i++) {
            swd_write_bit((data >> i) & 1);
        }
        // Пишем парность
        parity = __builtin_popcount(data) & 1;
        swd_write_bit(parity);

        return 0;
    }

    
    // 6. Turnaround обратно
    // swd_turnaround();
    // swd_io_dir_output();
   

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
        delay_us(2);
        SWCLK_HIGH();
        delay_us(2);
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
        delay_us(2);
        SWCLK_HIGH();
        delay_us(2);

        switch_sequence >>= 1;
    }
    
}

// ================= ВЫСОКОУРОВНЕВЫЕ ФУНКЦИИ =================
static uint32_t swd_read_idcode(void) {
    // Чтение IDCODE (DPACC, READ, адрес 0)
    uint8_t req = (1 << 0) |          // START
                  (0 << 1) |          // APnDP=0 (DP)
                  (1 << 2) |          // RnW=1 (Read)
                  (0 << 3) |          // A2=0
                  (0 << 4);           // A3=0
    
    return swd_transfer(req, 0);
}

static uint32_t swd_read_dp(uint8_t addr) {
    uint8_t req = (1 << 0) |          // START
                  (0 << 1) |          // APnDP=0 (DP)
                  (1 << 2) |          // RnW=1 (Read)
                  ((addr & 0x4) ? (1 << 3) : 0) |  // A2
                  ((addr & 0x8) ? (1 << 4) : 0);   // A3
    
    return swd_transfer(req, 0);
}

static void swd_write_dp(uint8_t addr, uint32_t data) {
    uint8_t req = (1 << 0) |          // START
                  (0 << 1) |          // APnDP=0 (DP)
                  (0 << 2) |          // RnW=0 (Write)
                  ((addr & 0x4) ? (1 << 3) : 0) |  // A2
                  ((addr & 0x8) ? (1 << 4) : 0);   // A3
    //FIXME: delete delay after test
    +
    swd_transfer(req, data);
}

static void swd_write_ap(uint8_t ap, uint8_t addr, uint32_t data) {
    // Сначала выбираем AP
    //swd_write_dp(DP_SELECT, (ap << 24) | ((addr & 0xF0) << 4));
    
    // Пишем в AP
    uint8_t req = (1 << 0) |          // START
                  (1 << 1) |          // APnDP=1 (AP)
                  (0 << 2) |          // RnW=0 (Write)
                  ((addr & 0x4) ? (1 << 3) : 0) |  // A2
                  ((addr & 0x8) ? (1 << 4) : 0);   // A3
    
    swd_transfer(req, data);
}

static uint32_t swd_read_ap(uint8_t ap, uint8_t addr) {
    // Сначала выбираем AP
    swd_write_dp(DP_SELECT, (ap << 24) | ((addr & 0xF0) << 4));
    
    // Читаем из AP 
    uint8_t req = (1 << 0) |          // START
                  (1 << 1) |          // APnDP=1 (AP)
                  (1 << 2) |          // RnW=1 (Read)
                  ((addr & 0x4) ? (1 << 3) : 0) |  // A2
                  ((addr & 0x8) ? (1 << 4) : 0);   // A3
    
    return swd_transfer(req, 0);
}


static int swd_init_debug(void) {
    // Инициализация отладочного порта
    
    swd_line_reset();
    
    swd_jtag_to_swd();
    swd_line_reset();
    // Читаем IDCODE
    uint32_t id = swd_read_idcode();
    if(id == 0xFFFFFFFF || id == 0) {
        return 0; // Ошибка
    }
    
    // Включаем питание отладки
    swd_write_dp(DP_CTRLSTAT, CSYSPWRUPREQ | CDBGPWRUPREQ);
    
    // Ждем подтверждения
    uint32_t timeout = 1000;
    while(timeout--) {
        uint32_t stat = swd_read_dp(DP_CTRLSTAT);
        if((stat & (CSYSPWRUPACK | CDBGPWRUPACK)) == (CSYSPWRUPACK | CDBGPWRUPACK)) {
            break;
        }
        delay_us(10);
    }
    
    // Выбираем AP #0
    swd_write_dp(DP_SELECT, 0);
    
    // Настраиваем AP (32-bit access, auto-increment)
     swd_write_ap(0, AP_CSW, 0x23000012);
    
    return 1; // Успех
}

static void swd_mem_write(uint32_t addr, uint32_t data) {
    // Записываем адрес
    swd_write_ap(0, AP_TAR, addr);
    // Записываем данные
    swd_write_ap(0, AP_DRW, data);
}

static uint32_t swd_mem_read(uint32_t addr) {
    // Записываем адрес
    swd_write_ap(0, AP_TAR, addr);
    // Читаем данные
    return swd_read_ap(0, AP_DRW);
}

// ================= Flash операции =================
static void flash_unlock(uint32_t flash_base) {
    // Разблокировка Flash контроллера
    swd_mem_write(flash_base + FLASH_KEYR, KEY1);
    swd_mem_write(flash_base + FLASH_KEYR, KEY2);
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

// ================= ОСНОВНАЯ ПРОГРАММА =================
int main(void) {
    // Настройка тактирования (72 МГц)
    RCC->CR |= RCC_CR_HSEON;                     // Включаем HSE
    while(!(RCC->CR & RCC_CR_HSERDY));           // Ждем готовности HSE

    FLASH->ACR = FLASH_ACR_LATENCY_2;            // 2 wait states для 72 МГц

    RCC->CFGR |= RCC_CFGR_PLLSRC;                // Источник PLL = HSE
    RCC->CFGR |= RCC_CFGR_PLLMULL9;              // Множитель PLL = 9

    RCC->CR |= RCC_CR_PLLON;                     // Включаем PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));           // Ждем готовности PLL

    RCC->CFGR |= RCC_CFGR_SW_PLL;                // Переключаемся на PLL
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ждем переключения

    SystemCoreClock = 72000000; 
   

    // Инициализация программатора
    swd_gpio_init();
    led_on();
    delay_ms(100);
    led_off();
   
    // Сброс целевой платы
    NRST_PORT->BRR = (1 << NRST_PIN);  // NRST = 0
    delay_ms(10);
    NRST_PORT->BSRR = (1 << NRST_PIN); // NRST = 1
    delay_ms(10);

    

   // Инициализация SWD
    if(!swd_init_debug()) {
        // Ошибка
        while(1) {
            led_toggle();
            delay_ms(300);
        
    
        }
    }

    
//     Читаем IDCODE
 //  FIXME: нужен ли здесь icode ? 
  // uint32_t id = swd_read_idcode();
    
// //     //адрес начала Flash для STM32F1)
    //   uint32_t flash_base = 0x40022000;
    //  uint32_t target_flash_start = 0x08000000;
    
    // Разблокируем Flash целевой платы
    //flash_unlock(flash_base);
    
//     // Стираем Flash
//     flash_erase_all(flash_base);
    
// //     // тестовых данных
//   // uint32_t test_data[] = {
//     // таблица векторов прерываний
//     0x200027FF,  // SP: 0x200027FF
//     0x08000201,  // Reset: 0x08000201
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x00000000,
//     0x00000000, 0x00000000, 0x00000000, 0x08000249, 0x08000249, 0x00000000,
//     0x08000249, 0x08000249,
    
//     // продолжение векторов 
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249, 0x08000249,
//     0x08000249, 0x08000249, 0x00000000, 0x08000249, 0x08000249, 0x00000000,
//     0x00000000, 0x08000249, 0x00000000, 0x08000249, 0x08000249, 0x00000000,
//     0x08000249, 0x08000249, 0x08000249, 0x00000000, 0x00000000, 0x00000000,
//     0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    
//     // Начало кода  мигающий светодиод
//     0x00000000, 0x00000000, 0xF1085FF8, 0x4B0348A3, 0xD2024283, 0xB1034B47,
//     0x20001847, 0x20000000, 0x00000000, 0x4B054806, 0xEBA31B1A, 0x024901D9,
//     0x4B03D210, 0x70471847, 0x20000000, 0x00000020, 0xB5100000, 0xB9437823,
//     0xFFDAFF06, 0x3008AF4B, 0x702313B1, 0x200010BD, 0x00000000, 0x49050294,
//     0x4008B508, 0xB1054B80, 0xFFCFBF49, 0x000000BF, 0x20000400, 0x4A150294,
//     0x1043F082, 0x5A699369, 0x027022F4, 0x125A6042, 0x6010F480, 0x33019B00,
//     0x9A009300, 0xF8DD420B, 0x524FF00B, 0x93002301, 0x42089A01, 0x4B06DD06,
//     0x1A61024F, 0x93EAE700, 0x019B3301, 0x40000193, 0x01001002, 0x00009F86,
//     0xFCFF7047, 0x480BFFF7, 0x4A0C490C, 0xE0022300, 0x33C450D4, 0xF9D304C4,
//     0x094A8C42, 0x2300094C, 0x6001E013, 0xFBD332A2, 0xFF0F8B00, 0x70B5FFF7,
//     0x4D0C2600, 0xA6420D4B, 0x260009D1, 0x19F800F0, 0xA6420A4D, 0x70BD0A4B,
//     0x479855F8, 0x36EE36E7, 0x479855F8, 0x08F236F2, 0x0008AC02, 0x0008AC02,
//     0x0008B002, 0xBC08BFF8, 0x4670479E, 0xBC08BFF8, 0x4670479E, 0x00087501,
//     0x00084D01
// };
    
//     uint32_t firmware_size = sizeof(test_data) / sizeof(test_data[0]);

//    for(uint32_t i = 0; i < 3; i++) {
//         flash_program_word(target_flash_start + i*4, test_data[i], flash_base);
//         led_toggle();
//         delay_ms(50);
//     }
    
//     // проверка данных из флеш и массива 
//     int ok = 1;
//    for(uint32_t i = 0; i < firmware_size; i++) {
//         uint32_t read = swd_mem_read(target_flash_start + i*4);
//         if(read != test_data[i]) {
//             ok = 0;
//             break;
//         }
//     }
    
//     // Результат
//     if(ok) {
//         // Успех 
//         while(1) {
//             led_on();
//             delay_ms(500);
//             led_off();
//             delay_ms(500);
//         }
//     } else {
//         // Ошибка 
//         while(1) {
//             led_on();
//             delay_ms(100);
//             led_off();
//             delay_ms(100);
//         }
//     }
}
