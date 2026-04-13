#include "../inc/oled.h"

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
    // Очищаем биты для PA0 и PA1
    GPIOA_CRL &= ~(0xFF << (SDA_PIN * 4));
    GPIOA_CRL &= ~(0xFF << (SCL_PIN * 4));
    
    // Устанавливаем режим: CNF=11 (Open-drain), MODE=11 (50MHz)
    GPIOA_CRL |= (0x07 << (SDA_PIN * 4));  // 0111 = Open-drain output, 50MHz
    GPIOA_CRL |= (0x07 << (SCL_PIN * 4));
    
    // Начальное состояние: высокий уровень
    GPIOA_BSRR = (1 << SDA_PIN) | (1 << SCL_PIN);
}

// Управление пинами через BSRR и BRR
void I2C_SDA_High(void) {
    GPIOA_BSRR = (1 << SDA_PIN);  // Set bit
}

void I2C_SDA_Low(void) {
    GPIOA_BRR = (1 << SDA_PIN);   // Reset bit
}

void I2C_SCL_High(void) {
    GPIOA_BSRR = (1 << SCL_PIN);
}

void I2C_SCL_Low(void) {
    GPIOA_BRR = (1 << SCL_PIN);
}

// Чтение состояния SDA (настройка на вход временно)
uint8_t I2C_Read_SDA(void) {
    // Временно переключаем в режим входа
    uint32_t old_crl = GPIOA_CRL;
    
    // Очищаем биты PA0 и настраиваем как Input floating
    GPIOA_CRL &= ~(0xF << (SDA_PIN * 4));
    GPIOA_CRL |= (0x04 << (SDA_PIN * 4));  // 0100 = Input floating
    
    // Небольшая задержка
    delay_us_oled(1);
    
    // Читаем состояние
    uint8_t state = (GPIOA_IDR >> SDA_PIN) & 0x01;
    
    // Возвращаем обратно в Output Open-drain
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
    // Отправка 8 бит
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
    
    // Получение ACK
    I2C_SDA_High();  // Отключаем SDA для приема ACK
   delay_us_oled(I2C_DELAY / 2);
    I2C_SCL_High();
    delay_us_oled(I2C_DELAY);
    
    uint8_t ack = I2C_Read_SDA();  // 0 = ACK, 1 = NACK
    
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
void OLED_Init(void) {
    delay_us_oled(100000);  // Задержка после включения питания
    
    OLED_WriteCommand(OLED_CMD_DISPLAY_OFF);
    OLED_WriteCommand(OLED_CMD_SET_DISP_CLK);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(OLED_CMD_SET_MULTIPLEX);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(OLED_CMD_SET_OFFSET);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(OLED_CMD_SET_START_LINE);
    OLED_WriteCommand(OLED_CMD_SET_CHARGE_PUMP);
    OLED_WriteCommand(0x14);  // Включить charge pump
    OLED_WriteCommand(OLED_CMD_SET_MEM_MODE);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(OLED_CMD_SET_SEG_REMAP);
    OLED_WriteCommand(OLED_CMD_SET_COM_SCAN);
    OLED_WriteCommand(OLED_CMD_SET_COM_PINS);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(OLED_CMD_SET_CONTRAST);
    OLED_WriteCommand(0x7F);
    OLED_WriteCommand(OLED_CMD_SET_PRECHARGE);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(OLED_CMD_SET_VCOM_DETECT);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(OLED_CMD_DISPLAY_ON);
    
    OLED_Clear();
    OLED_Update();
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
        OLED_WriteCommand(0xB0 + page);  // Установка страницы
        OLED_WriteCommand(0x00);         // Младший байт колонки
        OLED_WriteCommand(0x10);         // Старший байт колонки
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
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    int16_t e2;
    
    while(1) {
        OLED_SetPixel(x0, y0, color);
        if(x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if(e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if(e2 <= dx) {
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

// Простой шрифт 5x7 (только цифры)
static const uint8_t font5x7[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}   // 9
};

void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color) {
    if(ch < '0' || ch > '9') return;
    
    uint8_t index = ch - '0';
    for(uint8_t i = 0; i < 5; i++) {
        for(uint8_t j = 0; j < 7; j++) {
            if(font5x7[index][i] & (1 << j)) {
                OLED_SetPixel(x + i, y + j, color);
            }
        }
    }
}