#define USART_H
#ifdef USART_H

#include "stm32f1xx.h"
#include <stdint.h>
void TxHex8(uint8_t val);
void TxHex16(uint16_t val);
void TxHex32(uint32_t val); 
void TxLabelHex32(char *label, uint32_t val); 
void Tx2(uint8_t Symbol); 
uint8_t Rx2(); 
void TxStr2(uint8_t *pStr); 

#endif