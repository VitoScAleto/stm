#include "../inc/stm32f1xx.h"
#include <stdint.h>

#define SWCLK_PORT   GPIOA
#define SWCLK_PIN    8
#define SWDIO_PORT   GPIOA
#define SWDIO_PIN    9
#define NRST_PORT    GPIOB
#define NRST_PIN     0
#define LED_PORT     GPIOC
#define LED_PIN      13


#define SWDIO_HIGH() GPIOA->BSRR = (1 << SWDIO_PIN)
#define SWDIO_LOW() GPIOA->BSRR = (1 << (SWDIO_PIN+16))
#define SWCLK_HIGH() GPIOA->BSRR = (1 << SWCLK_PIN)
#define SWCLK_LOW() GPIOA->BSRR = (1 << (SWCLK_PIN+16))
#define NRST_HIGH() GPIOB->BSRR = (1 << NRST_PIN)
#define NRST_LOW() GPIOB->BSRR = (1 << (16+NRST_PIN))

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

static void init(void) {
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

int main(void)
{
    // --- 72 MHz ---
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR = FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = 72000000;

    init();
    dwt_init();   // <<< ВАЖНО

    while (1)
    {
        SWCLK_HIGH();
        delay_us(1);

        SWCLK_LOW();
        delay_us(1);
    }
}