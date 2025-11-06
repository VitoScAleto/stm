#include "../inc/stm32f1xx.h"

int main(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    
    // Настройка PC13 как выход (LED на BluePill)
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;
    
    while(1) {
        // Включить светодиод (инверсная логика)
        GPIOC->BSRR = GPIO_BSRR_BR13;  // Выключить
        for(volatile int i = 0; i < 100000; i++);
        
        // Выключить светодиод  
        GPIOC->BSRR = GPIO_BSRR_BS13;  // Включить
        for(volatile int i = 0; i < 100000; i++);
    }




}