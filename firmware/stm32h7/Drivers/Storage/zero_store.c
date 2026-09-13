/* Drivers/Storage: persistent zero read/write with magic + checksum. */
#include "zero_store.h"

#include "main.h"

static uint8_t sValid;
static float sAngleRad;

static uint32_t ZeroStore_ComputeCheck(uint32_t angle_word)
{
  return ZERO_STORE_MAGIC ^ ZERO_STORE_VERSION ^
         angle_word ^ ZERO_STORE_CHECK_XOR;
}

uint8_t ZeroStore_Init(void)
{
  const volatile uint32_t *saved =
      (const volatile uint32_t *)ZERO_STORE_FLASH_ADDRESS;
  union { float f; uint32_t u; } angle;

  angle.u = saved[2];
  sValid = ((saved[0] == ZERO_STORE_MAGIC) &&
            (saved[1] == ZERO_STORE_VERSION) &&
            (saved[3] == ZeroStore_ComputeCheck(angle.u))) ? 1U : 0U;
  sAngleRad = angle.f;
  return sValid;
}

uint8_t ZeroStore_HasValid(void) { return sValid; }
float ZeroStore_GetAngleRad(void) { return sAngleRad; }

void ZeroStore_SetAngleRad(float angle_rad)
{
  sAngleRad = angle_rad;
  sValid = 1U;
}

uint8_t ZeroStore_Write(float angle_rad)
{
  union { float f; uint32_t u; } angle;
  const volatile uint32_t *verify =
      (const volatile uint32_t *)ZERO_STORE_FLASH_ADDRESS;
  uint32_t sector_error = 0U;
  uint32_t check;
  FLASH_EraseInitTypeDef erase = {0};
  __align(32) static uint32_t flash_word[8];

  angle.f = angle_rad;
  check = ZeroStore_ComputeCheck(angle.u);
  flash_word[0] = ZERO_STORE_MAGIC;
  flash_word[1] = ZERO_STORE_VERSION;
  flash_word[2] = angle.u;
  flash_word[3] = check;
  flash_word[4] = flash_word[5] = flash_word[6] = flash_word[7] =
      0xFFFFFFFFUL;

  HAL_FLASH_Unlock();
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_1;
  erase.Sector = 7U;                       /* Bank1 Sector 7: 0x080E0000 */
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  if ((HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) ||
      (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                         ZERO_STORE_FLASH_ADDRESS,
                         (uint32_t)flash_word) != HAL_OK))
  {
    HAL_FLASH_Lock();
    return 1U;
  }
  HAL_FLASH_Lock();
  SCB_InvalidateDCache_by_Addr((uint32_t *)ZERO_STORE_FLASH_ADDRESS, 32U);
  if ((verify[0] != ZERO_STORE_MAGIC) ||
      (verify[1] != ZERO_STORE_VERSION) ||
      (verify[2] != angle.u) ||
      (verify[3] != check))
    return 1U;

  sAngleRad = angle_rad;
  sValid = 1U;
  return 0U;
}
