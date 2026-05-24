#include "../inc/oled_driver.h"

// Буфер для дисплея
uint8_t displayBuffer[OLED_PAGES][OLED_WIDTH];

// Задержка в микросекундах (приблизительная для 72MHz)
void delay_us_oled(uint32_t us) {
    for(uint32_t i = 0; i < us * 8; i++) {
        __asm("NOP");
    }
}

// Инициализация пинов I2C (регистровая)
void I2C_GPIO_Init(void) {
    // Включение тактирования GPIOA (бит 2 в RCC_APB2ENR)
    RCC_APB2ENR |= (1 << 2);
    
    // Настройка PA0 и PA1 как Output Open-drain, 50MHz
    GPIOA_CRL &= ~(0xFF << (SDA_PIN * 4));
    GPIOA_CRL &= ~(0xFF << (SCL_PIN * 4));
    
    GPIOA_CRL |= (0x07 << (SDA_PIN * 4));  // 0111 = Open-drain output, 50MHz
    GPIOA_CRL |= (0x07 << (SCL_PIN * 4));
    
    // Начальное состояние: высокий уровень
    GPIOA_BSRR = (1 << SDA_PIN) | (1 << SCL_PIN);
}

// Управление пинами
void I2C_SDA_High(void) {
    GPIOA_BSRR = (1 << SDA_PIN);
}

void I2C_SDA_Low(void) {
    GPIOA_BRR = (1 << SDA_PIN);
}

void I2C_SCL_High(void) {
    GPIOA_BSRR = (1 << SCL_PIN);
}

void I2C_SCL_Low(void) {
    GPIOA_BRR = (1 << SCL_PIN);
}

// Чтение состояния SDA
uint8_t I2C_Read_SDA(void) {
    uint32_t old_crl = GPIOA_CRL;
    
    GPIOA_CRL &= ~(0xF << (SDA_PIN * 4));
    GPIOA_CRL |= (0x04 << (SDA_PIN * 4));  // Input floating
    
    delay_us_oled(1);
    
    uint8_t state = (GPIOA_IDR >> SDA_PIN) & 0x01;
    
    GPIOA_CRL = old_crl;
    
    return state;
}

// Генерация START условия
void I2C_Start(void) {
    I2C_SDA_High();
    I2C_SCL_High();
    delay_us_oled(I2C_DELAY);
    I2C_SDA_Low();
    delay_us_oled(I2C_DELAY);
    I2C_SCL_Low();
    delay_us_oled(I2C_DELAY);
}

// Генерация STOP условия
void I2C_Stop(void) {
    I2C_SDA_Low();
    I2C_SCL_High();
    delay_us_oled(I2C_DELAY);
    I2C_SDA_High();
    delay_us_oled(I2C_DELAY);
}

// Отправка байта
uint8_t I2C_WriteByte(uint8_t data) {
    for(uint8_t i = 0; i < 8; i++) {
        if(data & 0x80) {
            I2C_SDA_High();
        } else {
            I2C_SDA_Low();
        }
        data <<= 1;
        delay_us_oled(I2C_DELAY / 2);
        I2C_SCL_High();
        delay_us_oled(I2C_DELAY);
        I2C_SCL_Low();
        delay_us_oled(I2C_DELAY / 2);
    }
    
    I2C_SDA_High();
    delay_us_oled(I2C_DELAY / 2);
    I2C_SCL_High();
    delay_us_oled(I2C_DELAY);
    
    uint8_t ack = I2C_Read_SDA();
    
    I2C_SCL_Low();
    delay_us_oled(I2C_DELAY / 2);
    
    return ack;
}

// Отправка команды на OLED
void OLED_WriteCommand(uint8_t cmd) {
    I2C_Start();
    I2C_WriteByte(OLED_ADDR);
    I2C_WriteByte(0x00);  // Control byte: команда
    I2C_WriteByte(cmd);
    I2C_Stop();
}

// Отправка данных
void OLED_WriteData(uint8_t data) {
    I2C_Start();
    I2C_WriteByte(OLED_ADDR);
    I2C_WriteByte(0x40);  // Control byte: данные
    I2C_WriteByte(data);
    I2C_Stop();
}

// Отправка массива данных
void OLED_WriteDataArray(uint8_t *data, uint16_t length) {
    I2C_Start();
    I2C_WriteByte(OLED_ADDR);
    I2C_WriteByte(0x40);  // Control byte: данные
    
    for(uint16_t i = 0; i < length; i++) {
        I2C_WriteByte(data[i]);
    }
    
    I2C_Stop();
}

// Инициализация OLED
void OLED_Init(void)
{
    delay_us_oled(100000);  //Пауза после подачи питания (обязательна для стабилизации OLED)

    OLED_WriteCommand(OLED_CMD_DISPLAY_OFF);//Выключаем дисплей перед настройкой (безопасный режим)

    OLED_WriteCommand(OLED_CMD_SET_DISP_CLK);
    OLED_WriteCommand(0x80);// Настройка частоты внутреннего генератора OLED (clock divide ratio / oscillator frequency)

    OLED_WriteCommand(OLED_CMD_SET_MULTIPLEX);
    OLED_WriteCommand(0x3F);// Устанавливаем мультиплекс (64 строки для 128x64 OLED)

    OLED_WriteCommand(OLED_CMD_SET_OFFSET);
    OLED_WriteCommand(0x00);// Смещение отображения по вертикали (обычно 0)

    OLED_WriteCommand(OLED_CMD_SET_START_LINE);// Начальная строка отображения = 0

    OLED_WriteCommand(OLED_CMD_SET_CHARGE_PUMP);
    OLED_WriteCommand(0x14);//  Включаем charge pump (внутренний повышающий преобразователь напряжения OLED)

    OLED_WriteCommand(OLED_CMD_SET_MEM_MODE);
    OLED_WriteCommand(0x00);
    // Режим памяти:
    // 0x00 = горизонтальный (обычно используемый)
    // 0x01 = вертикальный
    // 0x02 = page mode

    OLED_WriteCommand(OLED_CMD_SET_SEG_REMAP);
    //  Инверсия сегментов (горизонтальное зеркалирование по X)

    OLED_WriteCommand(OLED_CMD_SET_COM_SCAN);
    //  Инверсия строк (вертикальное зеркалирование)

    OLED_WriteCommand(OLED_CMD_SET_COM_PINS);
    OLED_WriteCommand(0x12);
    // Конфигурация COM пинов (для 128x64 OLED стандарт 0x12)

    OLED_WriteCommand(OLED_CMD_SET_CONTRAST);
    OLED_WriteCommand(0x7F);
    //  Контраст (0x00–0xFF), 0x7F — среднее значение

    OLED_WriteCommand(OLED_CMD_SET_PRECHARGE);
    OLED_WriteCommand(0xF1);
    //  Время зарядки пикселей (pre-charge period)

    OLED_WriteCommand(OLED_CMD_SET_VCOM_DETECT);
    OLED_WriteCommand(0x40);
    //  Уровень VCOMH (стабилизация напряжения дисплея)

    OLED_WriteCommand(OLED_CMD_DISPLAY_ON);
    //  Включаем дисплей

    OLED_Clear();   //  Очищаем буфер экрана
    OLED_Update();  //  Отправляем чистый экран на OLED
}

// Очистка буфера
void OLED_Clear(void) {
    for(uint8_t page = 0; page < OLED_PAGES; page++) {
        for(uint16_t col = 0; col < OLED_WIDTH; col++) {
            displayBuffer[page][col] = 0x00;
        }
    }
}

// Обновление дисплея из буфера
void OLED_Update(void) {
    for(uint8_t page = 0; page < OLED_PAGES; page++) {
        OLED_WriteCommand(0xB0 + page);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        OLED_WriteDataArray(displayBuffer[page], OLED_WIDTH);
    }
}

// Установка пикселя
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color) {
    if(x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    
    if(color) {
        displayBuffer[page][x] |= (1 << bit);
    } else {
        displayBuffer[page][x] &= ~(1 << bit);
    }
}

// Заполнение дисплея
void OLED_Fill(uint8_t color) {
    uint8_t fillValue = color ? 0xFF : 0x00;
    for(uint8_t page = 0; page < OLED_PAGES; page++) {
        for(uint16_t col = 0; col < OLED_WIDTH; col++) {
            displayBuffer[page][col] = fillValue;
        }
    }
    OLED_Update();
}

// Рисование линии (алгоритм Брезенхема)
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;
    
    while(1) {
        OLED_SetPixel(x0, y0, color);
        if(x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if(e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if(e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Рисование прямоугольника
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    OLED_DrawLine(x, y, x + w, y, color);
    OLED_DrawLine(x + w, y, x + w, y + h, color);
    OLED_DrawLine(x + w, y + h, x, y + h, color);
    OLED_DrawLine(x, y + h, x, y, color);
}

// Шрифт 5x7 для цифр
static const uint8_t font5x7[][5] = {
{0x00,0x00,0x00,0x00,0x00}, // space
{0x00,0x00,0x5F,0x00,0x00}, // !
{0x00,0x07,0x00,0x07,0x00}, // "
{0x14,0x7F,0x14,0x7F,0x14}, // #
{0x24,0x2A,0x7F,0x2A,0x12}, // $
{0x23,0x13,0x08,0x64,0x62}, // %
{0x36,0x49,0x55,0x22,0x50}, // &
{0x00,0x05,0x03,0x00,0x00}, // '
{0x00,0x1C,0x22,0x41,0x00}, // (
{0x00,0x41,0x22,0x1C,0x00}, // )
{0x14,0x08,0x3E,0x08,0x14}, // *
{0x08,0x08,0x3E,0x08,0x08}, // +
{0x00,0x50,0x30,0x00,0x00}, // ,
{0x08,0x08,0x08,0x08,0x08}, // -
{0x00,0x60,0x60,0x00,0x00}, // .
{0x20,0x10,0x08,0x04,0x02}, // /

{0x3E,0x51,0x49,0x45,0x3E}, // 0
{0x00,0x42,0x7F,0x40,0x00}, // 1
{0x42,0x61,0x51,0x49,0x46}, // 2
{0x21,0x41,0x45,0x4B,0x31}, // 3
{0x18,0x14,0x12,0x7F,0x10}, // 4
{0x27,0x45,0x45,0x45,0x39}, // 5
{0x3C,0x4A,0x49,0x49,0x30}, // 6
{0x01,0x71,0x09,0x05,0x03}, // 7
{0x36,0x49,0x49,0x49,0x36}, // 8
{0x06,0x49,0x49,0x29,0x1E}, // 9

{0x00,0x36,0x36,0x00,0x00}, // :
{0x00,0x56,0x36,0x00,0x00}, // ;
{0x08,0x14,0x22,0x41,0x00}, // <
{0x14,0x14,0x14,0x14,0x14}, // =
{0x00,0x41,0x22,0x14,0x08}, // >
{0x02,0x01,0x51,0x09,0x06}, // ?

{0x32,0x49,0x79,0x41,0x3E}, // @

{0x7E,0x11,0x11,0x11,0x7E}, // A
{0x7F,0x49,0x49,0x49,0x36}, // B
{0x3E,0x41,0x41,0x41,0x22}, // C
{0x7F,0x41,0x41,0x22,0x1C}, // D
{0x7F,0x49,0x49,0x49,0x41}, // E
{0x7F,0x09,0x09,0x09,0x01}, // F
{0x3E,0x41,0x49,0x49,0x7A}, // G
{0x7F,0x08,0x08,0x08,0x7F}, // H
{0x00,0x41,0x7F,0x41,0x00}, // I
{0x20,0x40,0x41,0x3F,0x01}, // J
{0x7F,0x08,0x14,0x22,0x41}, // K
{0x7F,0x40,0x40,0x40,0x40}, // L
{0x7F,0x02,0x0C,0x02,0x7F}, // M
{0x7F,0x04,0x08,0x10,0x7F}, // N
{0x3E,0x41,0x41,0x41,0x3E}, // O
{0x7F,0x09,0x09,0x09,0x06}, // P
{0x3E,0x41,0x51,0x21,0x5E}, // Q
{0x7F,0x09,0x19,0x29,0x46}, // R
{0x46,0x49,0x49,0x49,0x31}, // S
{0x01,0x01,0x7F,0x01,0x01}, // T
{0x3F,0x40,0x40,0x40,0x3F}, // U
{0x1F,0x20,0x40,0x20,0x1F}, // V
{0x3F,0x40,0x38,0x40,0x3F}, // W
{0x63,0x14,0x08,0x14,0x63}, // X
{0x07,0x08,0x70,0x08,0x07}, // Y
{0x61,0x51,0x49,0x45,0x43}, // Z

{0x00,0x7F,0x41,0x41,0x00}, // [
{0x02,0x04,0x08,0x10,0x20}, // '\'
{0x00,0x41,0x41,0x7F,0x00}, // ]
{0x04,0x02,0x01,0x02,0x04}, // ^
{0x40,0x40,0x40,0x40,0x40}, // _

{0x00,0x01,0x02,0x04,0x00}, // `
{0x20,0x54,0x54,0x54,0x78}, // a
{0x7F,0x48,0x44,0x44,0x38}, // b
{0x38,0x44,0x44,0x44,0x20}, // c
{0x38,0x44,0x44,0x48,0x7F}, // d
{0x38,0x54,0x54,0x54,0x18}, // e
{0x08,0x7E,0x09,0x01,0x02}, // f
{0x0C,0x52,0x52,0x52,0x3E}, // g
{0x7F,0x08,0x04,0x04,0x78}, // h
{0x00,0x44,0x7D,0x40,0x00}, // i
{0x20,0x40,0x44,0x3D,0x00}, // j
{0x7F,0x10,0x28,0x44,0x00}, // k
{0x00,0x41,0x7F,0x40,0x00}, // l
{0x7C,0x04,0x18,0x04,0x78}, // m
{0x7C,0x08,0x04,0x04,0x78}, // n
{0x38,0x44,0x44,0x44,0x38}, // o
{0x7C,0x14,0x14,0x14,0x08}, // p
{0x08,0x14,0x14,0x18,0x7C}, // q
{0x7C,0x08,0x04,0x04,0x08}, // r
{0x48,0x54,0x54,0x54,0x20}, // s
{0x04,0x3F,0x44,0x40,0x20}, // t
{0x3C,0x40,0x40,0x20,0x7C}, // u
{0x1C,0x20,0x40,0x20,0x1C}, // v
{0x3C,0x40,0x30,0x40,0x3C}, // w
{0x44,0x28,0x10,0x28,0x44}, // x
{0x0C,0x50,0x50,0x50,0x3C}, // y
{0x44,0x64,0x54,0x4C,0x44}  // z
};

// Вывод символа
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color) {
    if (ch < 32 || ch > 126) return;

    const uint8_t *bitmap = font5x7[ch - 32];

    for(uint8_t i = 0; i < 5; i++) {
        for(uint8_t j = 0; j < 8; j++) {
            if(bitmap[i] & (1 << j)) {
                OLED_SetPixel(x + i, y + j, color);
            }
        }
    }
}

// ДОБАВЛЕННАЯ ФУНКЦИЯ: Вывод строки
void OLED_DrawString(uint8_t x, uint8_t y, const char* str, uint8_t color) {
    uint8_t current_x = x;
    uint8_t current_y = y;
    
    while(*str) {
        if(*str == '\n') {
            current_x = x;
            current_y += 8;
            str++;
            continue;
        }
        
        OLED_DrawChar(current_x, current_y, *str, color);
        current_x += 6;  // 5 ширина символа + 1 пробел
        str++;
        
        if(current_x + 6 > OLED_WIDTH) {
            current_x = x;
            current_y += 8;
        }
        
        if(current_y + 8 > OLED_HEIGHT) break;
    }
}