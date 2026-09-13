#ifndef BSP_UART1_H
#define BSP_UART1_H

#include <stdint.h>

void BspUart1_Init(void);
void BspUart1_IrqHandler(void);
uint8_t BspUart1_ReadByte(uint8_t *byte);
uint8_t BspUart1_Write(const uint8_t *data, uint16_t length);
uint32_t BspUart1_GetRxOverflowCount(void);
uint32_t BspUart1_GetTxDropCount(void);

#endif
