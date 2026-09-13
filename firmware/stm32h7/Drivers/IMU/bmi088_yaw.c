/* Task-facing calibration adapter. The BMI088 protocol is DM reference code. */
#include "bmi088_yaw.h"
#include "bmi088_board.h"

#define BMI088_BIAS_SAMPLE_COUNT 1000U
#define RAD_TO_DPS 57.2957795f

uint8_t BMI088_Yaw_Init(BMI088_Yaw_t *imu)
{
  float gyro[3];
  float accel[3];
  float temperature;

  *imu = (BMI088_Yaw_t){0};
  if (BMI088_Board_Init() != 0U)
  {
    imu->status = BMI088_YAW_CONFIG_ERROR;
    return imu->status;
  }
  if (BMI088_Board_Read(gyro, accel, &temperature) != 0U)
  {
    imu->status = BMI088_YAW_SPI_ERROR;
    return imu->status;
  }
  imu->chip_id = 0x0FU;
  imu->configured = 1U;
  imu->status = BMI088_YAW_OK;
  return BMI088_YAW_OK;
}

uint8_t BMI088_Yaw_Update(BMI088_Yaw_t *imu, float dt_s)
{
  float gyro[3];
  float accel[3];
  float temperature;
  float x_dps;
  float y_dps;
  float z_dps;

  if ((imu->status != BMI088_YAW_OK) ||
      (BMI088_Board_Read(gyro, accel, &temperature) != 0U))
  {
    imu->status = BMI088_YAW_SPI_ERROR;
    return imu->status;
  }
  /* The official driver exposes gyro[] in rad/s. */
  x_dps = gyro[0] * RAD_TO_DPS;
  y_dps = gyro[1] * RAD_TO_DPS;
  z_dps = gyro[2] * RAD_TO_DPS;
  if (imu->bias_samples < BMI088_BIAS_SAMPLE_COUNT)
  {
    imu->bias_x_dps += (x_dps - imu->bias_x_dps) / (float)(imu->bias_samples + 1U);
    imu->bias_y_dps += (y_dps - imu->bias_y_dps) / (float)(imu->bias_samples + 1U);
    imu->bias_dps += (z_dps - imu->bias_dps) / (float)(imu->bias_samples + 1U);
    imu->bias_samples++;
    if (imu->bias_samples == BMI088_BIAS_SAMPLE_COUNT)
      imu->bias_ready = 1U;
  }
  imu->gyro_x_dps = x_dps - imu->bias_x_dps;
  imu->gyro_y_dps = y_dps - imu->bias_y_dps;
  imu->gyro_z_dps = z_dps - imu->bias_dps;
  imu->yaw_rate_dps = imu->gyro_z_dps;
  if (imu->bias_ready != 0U)
    imu->yaw_angle_deg += imu->yaw_rate_dps * dt_s;
  imu->sample_count++;
  return BMI088_YAW_OK;
}
