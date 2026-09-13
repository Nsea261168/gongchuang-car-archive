/* Drivers/Storage: persistent calibration zero in STM32H723 Bank1 Sector 7. */
#ifndef ZERO_STORE_H
#define ZERO_STORE_H

#include <stdint.h>

#define ZERO_STORE_FLASH_ADDRESS  0x080E0000UL
#define ZERO_STORE_MAGIC          0x5A45524FUL
#define ZERO_STORE_VERSION        0x00000001UL
#define ZERO_STORE_CHECK_XOR      0xA5A55A5AUL

/* Reads and validates the flash record; returns the valid flag (0 or 1). */
uint8_t ZeroStore_Init(void);
uint8_t ZeroStore_HasValid(void);
float ZeroStore_GetAngleRad(void);
/* Updates the in-RAM copy (call after a successful ZeroStore_Write). */
void ZeroStore_SetAngleRad(float angle_rad);
/* Erases the sector, programs and verifies. Returns 0 on success. */
uint8_t ZeroStore_Write(float angle_rad);

#endif
