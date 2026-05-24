#define OLED_UI_H
#ifdef OLED_UI_H

#include <stdint.h>
#include "stm32f1xx.h"
#include "at25df321a_firmware.h"
#include "oled_driver.h"
#include <stdio.h>

#define BTN_UP_PIN      10
#define BTN_DOWN_PIN    11
#define BTN_OK_PIN      12
#define BTN_BACK_PIN    13

#define BTN_UP_MASK     (1u << BTN_UP_PIN)
#define BTN_DOWN_MASK   (1u << BTN_DOWN_PIN)
#define BTN_OK_MASK     (1u << BTN_OK_PIN)
#define BTN_BACK_MASK   (1u << BTN_BACK_PIN)


void Buttons_Init(void);

uint8_t Button_IsPressed(uint32_t mask);

void OLED_Status(const char* a, const char* b);

void OLED_ShowMainMenu(void);

void OLED_ShowFirmwareSelect(void);

void OLED_DoFlashSelected(void);

void OLED_DoVerifySelected(void);

void OLED_DoMemoryInfo(void);

void OLED_MenuTask(void);

#endif