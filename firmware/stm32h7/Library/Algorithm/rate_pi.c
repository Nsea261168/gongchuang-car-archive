/* Library/Algorithm: rate PI. */
#include "rate_pi.h"

static float RatePI_Clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void RatePI_Init(RatePI_t *pi, float kp, float ki, float output_limit)
{
  pi->kp = kp;
  pi->ki = ki;
  pi->kd = 0.0f;
  pi->output_limit = output_limit;
  pi->integral_limit = output_limit;
  pi->output_lpf_rc_s = 0.0f;
  pi->integral = 0.0f;
  pi->output = 0.0f;
  pi->last_measurement = 0.0f;
  pi->last_output = 0.0f;
  pi->blocked_count = 0U;
}

void RatePI_SetAdvanced(RatePI_t *pi, float kd, float integral_limit,
                        float output_lpf_rc_s)
{
  pi->kd = kd;
  pi->integral_limit = integral_limit;
  pi->output_lpf_rc_s = output_lpf_rc_s;
}

void RatePI_Reset(RatePI_t *pi)
{
  pi->integral = 0.0f;
  pi->output = 0.0f;
  pi->last_measurement = 0.0f;
  pi->last_output = 0.0f;
  pi->blocked_count = 0U;
}

float RatePI_UpdateWithFeedForward(RatePI_t *pi, float reference,
                                   float measurement, float feed_forward,
                                   float dt_s)
{
  float error = reference - measurement;
  float derivative = (dt_s > 0.000001f) ?
                     pi->kd * (pi->last_measurement - measurement) / dt_s : 0.0f;
  float candidate_integral = pi->integral + pi->ki * error * dt_s;
  float candidate_output;
  float raw_output;

  candidate_integral = RatePI_Clamp(candidate_integral, pi->integral_limit);
  candidate_output = pi->kp * error + candidate_integral + derivative + feed_forward;

  /* Conditional integration prevents wind-up whenever current is clamped. */
  if ((candidate_output < pi->output_limit) &&
      (candidate_output > -pi->output_limit))
    pi->integral = candidate_integral;

  raw_output = RatePI_Clamp(pi->kp * error + pi->integral + derivative + feed_forward,
                            pi->output_limit);
  if (pi->output_lpf_rc_s > 0.0f)
    pi->output = raw_output * dt_s / (pi->output_lpf_rc_s + dt_s) +
                 pi->last_output * pi->output_lpf_rc_s / (pi->output_lpf_rc_s + dt_s);
  else
    pi->output = raw_output;
  pi->last_measurement = measurement;
  pi->last_output = pi->output;
  return pi->output;
}

float RatePI_Update(RatePI_t *pi, float reference, float measurement,
                    float dt_s)
{
  return RatePI_UpdateWithFeedForward(pi, reference, measurement, 0.0f, dt_s);
}
