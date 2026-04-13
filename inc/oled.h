#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdlib.h>
#include "../inc/stm32f1xx.h"

// Регистры GPIOA
#define GPIOA_CRL      (*((volatile uint32_t*)0x40010800))
#define GPIOA_CRH      (*((volatile uint32_t*)0x40010804))
#define GPIOA_IDR      (*((volatile uint32_t*)0x40010808))
#define GPIOA_ODR      (*((volatile uint32_t*)0x4001080C))
#define GPIOA_BSRR     (*((volatile uint32_t*)0x40010810))
#define GPIOA_BRR      (*((volatile uint32_t*)0x40010814))

// Регистры RCC
#define RCC_APB2ENR    (*((volatile uint32_t*)0x40021018))

// Биты для GPIOA
#define SDA_PIN        0      // PA0
#define SCL_PIN        1      // PA1

// Режимы GPIO (CNF+MOD)
#define GPIO_OUT_OD_50MHZ  (0x07 << (SDA_PIN * 4))  // Output Open-drain, 50MHz
#define GPIO_OUT_OD_50MHZ_SCL (0x07 << (SCL_PIN * 4))

// Адрес SSD1306
#define OLED_ADDR      0x78

// Команды SSD1306
#define OLED_CMD_DISPLAY_OFF     0xAE
#define OLED_CMD_DISPLAY_ON      0xAF
#define OLED_CMD_SET_DISP_CLK    0xD5
#define OLED_CMD_SET_MULTIPLEX   0xA8
#define OLED_CMD_SET_OFFSET      0xD3
#define OLED_CMD_SET_START_LINE  0x40
#define OLED_CMD_SET_CHARGE_PUMP 0x8D
#define OLED_CMD_SET_MEM_MODE    0x20
#define OLED_CMD_SET_SEG_REMAP   0xA1
#define OLED_CMD_SET_COM_SCAN    0xC8
#define OLED_CMD_SET_COM_PINS    0xDA
#define OLED_CMD_SET_CONTRAST    0x81
#define OLED_CMD_SET_PRECHARGE   0xD9
#define OLED_CMD_SET_VCOM_DETECT 0xDB

// Размеры дисплея
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

#define I2C_DELAY    5

// Глобальный буфер
extern uint8_t displayBuffer[OLED_PAGES][OLED_WIDTH];

// Прототипы функций
void delay_us_oled(uint32_t us);
void I2C_GPIO_Init(void);
void I2C_SDA_High(void);
void I2C_SDA_Low(void);
void I2C_SCL_High(void);
void I2C_SCL_Low(void);
uint8_t I2C_Read_SDA(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WriteByte(uint8_t data);
void OLED_WriteCommand(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_WriteDataArray(uint8_t *data, uint16_t length);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_Fill(uint8_t color);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color);
void OLED_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color);  // ДОБАВИТЬ ЭТУ СТРОКУ

#endif