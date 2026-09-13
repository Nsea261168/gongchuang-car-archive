/* Drivers/IMU: yaw feedback interface. */
#ifndef BMI088_YAW_H
#define BMI088_YAW_H

#include <stdint.h>

typedef enum
{
  BMI088_YAW_OK = 0,
  BMI088_YAW_SPI_ERROR = 1,
  BMI088_YAW_CHIP_ID_ERROR = 2,
  BMI088_YAW_CONFIG_ERROR = 3
} BMI088_YawStatus_t;

typedef struct
{
  uint8_t status;
  uint8_t chip_id;
  uint8_t configured;
  uint8_t bias_ready;
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float bias_x_dps;
  float bias_y_dps;
  float bias_dps;
  float yaw_rate_dps;
  float yaw_angle_deg;
  float peak_abs_gyro_x_dps;
  float peak_abs_gyro_y_dps;
  float peak_abs_yaw_rate_dps;
  uint16_t bias_samples;
  uint32_t sample_count;
} BMI088_Yaw_t;

uint8_t BMI088_Yaw_Init(BMI088_Yaw_t *imu);
uint8_t BMI088_Yaw_Update(BMI088_Yaw_t *imu, float dt_s);

#endif
