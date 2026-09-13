#ifndef BSP_UART7_H
#define BSP_UART7_H

#include <stdint.h>

void BspUart7_Init(void);
void BspUart7_IrqHandler(void);
uint8_t BspUart7_ReadByte(uint8_t *byte);
uint8_t BspUart7_Write(const uint8_t *data, uint16_t length);
uint32_t BspUart7_GetRxOverflowCount(void);
uint32_t BspUart7_GetTxDropCount(void);

#endif
