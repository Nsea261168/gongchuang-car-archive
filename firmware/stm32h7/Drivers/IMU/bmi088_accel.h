/* BMI088 accelerometer driver for DM-MC-Board02 (SPI2, CS=PC0). */
#ifndef BMI088_ACCEL_H
#define BMI088_ACCEL_H

#include <stdint.h>

typedef enum
{
  BMI088_ACCEL_OK = 0,
  BMI088_ACCEL_SPI_ERROR = 1,
  BMI088_ACCEL_CHIP_ID_ERROR = 2,
  BMI088_ACCEL_CONFIG_ERROR = 3
} BMI088_AccelStatus_t;

typedef struct
{
  uint8_t status;
  uint8_t chip_id;
  uint8_t configured;
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  float x_mps2;
  float y_mps2;
  float z_mps2;
} BMI088_Accel_t;

uint8_t BMI088_Accel_Init(BMI088_Accel_t *imu);
uint8_t BMI088_Accel_Update(BMI088_Accel_t *imu);

#endif
