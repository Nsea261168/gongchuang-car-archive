#include "bmi088_board.h"
#include "BMI088driver.h"

static uint8_t initialized;
static uint8_t init_error;

uint8_t BMI088_Board_Init(void)
{
  if (initialized == 0U)
  {
    init_error = BMI088_init();
    if (init_error == BMI088_NO_ERROR)
      initialized = 1U;
  }
  return init_error;
}

uint8_t BMI088_Board_Read(float gyro[3], float accel[3], float *temperature)
{
  if ((initialized == 0U) && (BMI088_Board_Init() != BMI088_NO_ERROR))
    return init_error;
  BMI088_read(gyro, accel, temperature);
  return BMI088_NO_ERROR;
}
