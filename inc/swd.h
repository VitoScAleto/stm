#define SWD_H
#ifdef SWD_H

#include "stm32f1xx.h"
void TxHex8(uint8_t val);
void TxHex16(uint16_t val);
void TxHex32(uint32_t val); 
void TxLabelHex32(char *label, uint32_t val);
void USART2_Init(void);
void Tx2(uint8_t Symbol);
uint8_t Rx2(void);
void TxStr2(uint8_t *pStr);

#endif