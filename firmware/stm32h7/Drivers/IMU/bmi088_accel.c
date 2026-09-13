/* Task-facing adapter only. All BMI088 bus access/configuration is in
 * Damiao's CtrBoard-H7_IMU reference BMI088driver.c. */
#include "bmi088_accel.h"
#include "bmi088_board.h"

uint8_t BMI088_Accel_Init(BMI088_Accel_t *imu)
{
  float gyro[3];
  float accel[3];
  float temperature;
  uint8_t error;

  *imu = (BMI088_Accel_t){0};
  error = BMI088_Board_Init();
  if (error != 0U)
  {
    imu->status = BMI088_ACCEL_CONFIG_ERROR;
    return imu->status;
  }
  error = BMI088_Board_Read(gyro, accel, &temperature);
  if (error != 0U)
  {
    imu->status = BMI088_ACCEL_SPI_ERROR;
    return imu->status;
  }
  imu->configured = 1U;
  imu->chip_id = 0x1EU;
  imu->status = BMI088_ACCEL_OK;
  return BMI088_ACCEL_OK;
}

uint8_t BMI088_Accel_Update(BMI088_Accel_t *imu)
{
  float gyro[3];
  float accel[3];
  float temperature;

  if (BMI088_Board_Read(gyro, accel, &temperature) != 0U)
  {
    imu->status = BMI088_ACCEL_SPI_ERROR;
    return imu->status;
  }
  /* DM reference returns acceleration in m/s^2. */
  imu->x_mps2 = accel[0];
  imu->y_mps2 = accel[1];
  imu->z_mps2 = accel[2];
  imu->raw_x = (int16_t)(accel[0] / 0.0008974358974f);
  imu->raw_y = (int16_t)(accel[1] / 0.0008974358974f);
  imu->raw_z = (int16_t)(accel[2] / 0.0008974358974f);
  imu->status = BMI088_ACCEL_OK;
  return BMI088_ACCEL_OK;
}
