#include "../inc/swd_driver.h"
#include <stdint.h>
#include <stdio.h>
#include "../inc/oled_driver.h"
#include "../inc/oled_ui.h"
#include "../inc/at25df321a_firmware.h"
#include "../inc/uart_firmware_handler.h"
#include "../inc/at25df321a.h"
#include "../inc/stm32f1xx.h"
#include "../inc/swd_driver.h"
#include <stdint.h>
#include <string.h>

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
