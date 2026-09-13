/* Application adapter around Damiao's unmodified BMI088 and Mahony modules. */
#include "bmi088_attitude.h"

#include "bmi088_board.h"
#include "MahonyAHRS.h"

#include <math.h>

#define ATTITUDE_BIAS_SAMPLES       1000U
#define ATTITUDE_GRAVITY_MPS2       9.80665f
#define ATTITUDE_RAD_TO_DEG         57.2957795f
#define ATTITUDE_BASE_TWO_KP        1.0f
#define ATTITUDE_ACCEL_FULL_ERROR   0.08f
#define ATTITUDE_ACCEL_ZERO_ERROR   0.30f
#define ATTITUDE_MAX_SUBSTEPS       25U
#define ATTITUDE_MAX_STEP_S         0.002f

static float ClampFloat(float value, float lo, float hi)
{
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static void InitQuaternionFromGravity(BMI088_Attitude_t *imu)
{
  float ax = imu->accel_mps2[0];
  float ay = imu->accel_mps2[1];
  float az = imu->accel_mps2[2];
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);

  imu->quaternion[0] = cr * cp;
  imu->quaternion[1] = sr * cp;
  imu->quaternion[2] = cr * sp;
  imu->quaternion[3] = -sr * sp;
}

static void QuaternionToEuler(BMI088_Attitude_t *imu)
{
  const float *q = imu->quaternion;
  float pitch_sin = -2.0f * (q[1] * q[3] - q[0] * q[2]);
  float roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                      2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
  float pitch = asinf(ClampFloat(pitch_sin, -1.0f, 1.0f));
  float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                     2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);

  /* Preserve the board-axis convention already verified in the old task. */
  imu->roll_deg = -roll * ATTITUDE_RAD_TO_DEG;
  imu->pitch_deg = pitch * ATTITUDE_RAD_TO_DEG;
  imu->yaw_deg = yaw * ATTITUDE_RAD_TO_DEG;
}

uint8_t BMI088_Attitude_Init(BMI088_Attitude_t *imu)
{
  uint8_t i;
  *imu = (BMI088_Attitude_t){0};
  imu->quaternion[0] = 1.0f;
  imu->accel_weight = 1.0f;
  imu->motion_state = BMI088_MOTION_CALIBRATING;
  for (i = 0U; i < 3U; i++) imu->gyro_bias_rps[i] = 0.0f;
  if (BMI088_Board_Init() != 0U)
  {
    imu->status = BMI088_ATTITUDE_CONFIG_ERROR;
    return imu->status;
  }
  imu->status = BMI088_ATTITUDE_OK;
  return BMI088_ATTITUDE_OK;
}

uint8_t BMI088_Attitude_Update(BMI088_Attitude_t *imu, float dt_s)
{
  float gyro[3];
  float accel[3];
  float corrected[3];
  float accel_error;
  float gyro_abs_dps;
  float step_dt;
  uint8_t steps;
  uint8_t step;
  uint8_t axis;

  if ((imu->status != BMI088_ATTITUDE_OK) ||
      (BMI088_Board_Read(gyro, accel, &imu->temperature_c) != 0U))
  {
    imu->status = BMI088_ATTITUDE_READ_ERROR;
    return imu->status;
  }

  dt_s = ClampFloat(dt_s, 0.00025f, 0.050f);
  imu->last_dt_s = dt_s;
  for (axis = 0U; axis < 3U; axis++) imu->accel_mps2[axis] = accel[axis];
  imu->accel_norm_mps2 = sqrtf(accel[0] * accel[0] +
                               accel[1] * accel[1] +
                               accel[2] * accel[2]);
  imu->sample_count++;

  if (imu->bias_samples < ATTITUDE_BIAS_SAMPLES)
  {
    for (axis = 0U; axis < 3U; axis++)
      imu->gyro_bias_rps[axis] +=
          (gyro[axis] - imu->gyro_bias_rps[axis]) /
          (float)(imu->bias_samples + 1U);
    imu->bias_samples++;
    imu->motion_state = BMI088_MOTION_CALIBRATING;
    if (imu->bias_samples == ATTITUDE_BIAS_SAMPLES)
    {
      imu->bias_ready = 1U;
      InitQuaternionFromGravity(imu);
      QuaternionToEuler(imu);
    }
    return BMI088_ATTITUDE_OK;
  }

  for (axis = 0U; axis < 3U; axis++)
  {
    corrected[axis] = gyro[axis] - imu->gyro_bias_rps[axis];
    imu->gyro_dps[axis] = corrected[axis] * ATTITUDE_RAD_TO_DEG;
  }
  /* Match the displayed roll-axis sign to the physical board convention. */
  imu->gyro_dps[0] = -imu->gyro_dps[0];

  accel_error = fabsf(imu->accel_norm_mps2 - ATTITUDE_GRAVITY_MPS2) /
                ATTITUDE_GRAVITY_MPS2;
  if (accel_error <= ATTITUDE_ACCEL_FULL_ERROR)
    imu->accel_weight = 1.0f;
  else if (accel_error >= ATTITUDE_ACCEL_ZERO_ERROR)
    imu->accel_weight = 0.0f;
  else
    imu->accel_weight = (ATTITUDE_ACCEL_ZERO_ERROR - accel_error) /
                        (ATTITUDE_ACCEL_ZERO_ERROR - ATTITUDE_ACCEL_FULL_ERROR);

  gyro_abs_dps = fabsf(imu->gyro_dps[0]) + fabsf(imu->gyro_dps[1]) +
                 fabsf(imu->gyro_dps[2]);
  if (imu->accel_weight <= 0.05f)
    imu->motion_state = BMI088_MOTION_IMPACT;
  else if ((imu->accel_weight < 0.95f) || (gyro_abs_dps > 8.0f))
    imu->motion_state = BMI088_MOTION_MOVING;
  else
    imu->motion_state = BMI088_MOTION_STABLE;

  steps = (uint8_t)(dt_s / ATTITUDE_MAX_STEP_S);
  if (((float)steps * ATTITUDE_MAX_STEP_S) < dt_s) steps++;
  if (steps == 0U) steps = 1U;
  if (steps > ATTITUDE_MAX_SUBSTEPS) steps = ATTITUDE_MAX_SUBSTEPS;
  step_dt = dt_s / (float)steps;

  for (step = 0U; step < steps; step++)
  {
    float time_scale = step_dt * 1000.0f;
    twoKp = ATTITUDE_BASE_TWO_KP * time_scale * imu->accel_weight;
    twoKi = 0.0f;
    MahonyAHRSupdateIMU(imu->quaternion,
        corrected[0] * time_scale,
        corrected[1] * time_scale,
        corrected[2] * time_scale,
        (imu->accel_weight > 0.0f) ? accel[0] : 0.0f,
        (imu->accel_weight > 0.0f) ? accel[1] : 0.0f,
        (imu->accel_weight > 0.0f) ? accel[2] : 0.0f);
  }
  twoKp = ATTITUDE_BASE_TWO_KP;
  QuaternionToEuler(imu);
  return BMI088_ATTITUDE_OK;
}
