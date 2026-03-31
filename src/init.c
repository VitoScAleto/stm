#include "../inc/swd.h"

/*PA2 (TX2) и PA3 (RX2)*/
void USART2_Init(void)
{
    // 1. Включаем тактирование GPIOA и USART2
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // 2. Настройка PA2 (TX) как Alternate Function Push-Pull
    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0; // Output 50 MHz
    GPIOA->CRL |= GPIO_CRL_CNF2_1;                     // AF Push-Pull

    // 3. Настройка PA3 (RX) как вход (Floating input)
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    GPIOA->CRL |= GPIO_CRL_CNF3_0; // Floating input

    // 4. Настройка скорости (baudrate)
    // 72 MHz / 16 / 115200 ≈ 39.0625
    USART2->BRR = 72000000 / 115200;

    // 5. Включаем USART
    USART2->CR1 =
        USART_CR1_TE |   // передача
        USART_CR1_RE |   // приём
        USART_CR1_UE |   // включить USART
        USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART2_IRQn);
}
