#ifndef BMI088_ATTITUDE_H
#define BMI088_ATTITUDE_H

#include <stdint.h>

typedef enum
{
  BMI088_ATTITUDE_OK = 0,
  BMI088_ATTITUDE_CONFIG_ERROR,
  BMI088_ATTITUDE_READ_ERROR
} BMI088_AttitudeStatus_t;

typedef enum
{
  BMI088_MOTION_CALIBRATING = 0,
  BMI088_MOTION_STABLE,
  BMI088_MOTION_MOVING,
  BMI088_MOTION_IMPACT
} BMI088_MotionState_t;

typedef struct
{
  float quaternion[4];
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float gyro_dps[3];
  float accel_mps2[3];
  float gyro_bias_rps[3];
  float accel_norm_mps2;
  float accel_weight;
  float temperature_c;
  float last_dt_s;
  uint32_t sample_count;
  uint16_t bias_samples;
  uint8_t bias_ready;
  uint8_t status;
  uint8_t motion_state;
} BMI088_Attitude_t;

uint8_t BMI088_Attitude_Init(BMI088_Attitude_t *imu);
uint8_t BMI088_Attitude_Update(BMI088_Attitude_t *imu, float dt_s);

#endif
