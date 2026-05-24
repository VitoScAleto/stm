#include "../inc/usart.h"

void TxHex8(uint8_t val) {
    char hex[] = "0123456789ABCDEF";

    Tx2(hex[(val >> 4) & 0xF]);
    Tx2(hex[val & 0xF]);
}

void TxHex16(uint16_t val) {
    TxHex8((val >> 8) & 0xFF);
    TxHex8(val & 0xFF);
}

void TxHex32(uint32_t val) {
    // Выводим 8 символов HEX, ведущие нули тоже будут
    for(int i = 7; i >= 0; i--) {
        uint8_t nibble = (val >> (i * 4)) & 0xF;
        if(nibble < 10) 
            Tx2('0' + nibble);
        else 
            Tx2('A' + (nibble - 10));
    }
}
void TxLabelHex32(char *label, uint32_t val) {
    TxStr2((uint8_t*)label);
    TxStr2((uint8_t*)"0x");
    TxHex32(val);
    TxStr2((uint8_t*)"\r\n");
}
void Tx2(uint8_t Symbol) {
    while ((USART2->SR & USART_SR_TXE) == 0) {};
    USART2->DR = Symbol;
}

uint8_t Rx2() {
    while ((USART2->SR & USART_SR_RXNE) == 0) {};
    return USART2->DR;
}

void TxStr2(uint8_t *pStr) {
    uint8_t i = 0;
    while (pStr[i] != 0) {
        Tx2(pStr[i++]);
    }
}