/* Library/Algorithm: rate PI interface. */
#ifndef RATE_PI_H
#define RATE_PI_H

#include <stdint.h>

/* Generic rate PID: derivative on measurement, output low-pass filtered. */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float output_lpf_rc_s;
  float integral;
  float output;
  float last_measurement;
  float last_output;
  uint16_t blocked_count;
} RatePI_t;

void RatePI_Init(RatePI_t *pi, float kp, float ki, float output_limit);
void RatePI_SetAdvanced(RatePI_t *pi, float kd, float integral_limit,
                        float output_lpf_rc_s);
void RatePI_Reset(RatePI_t *pi);
float RatePI_Update(RatePI_t *pi, float reference, float measurement,
                    float dt_s);
float RatePI_UpdateWithFeedForward(RatePI_t *pi, float reference,
                                   float measurement, float feed_forward,
                                   float dt_s);

#endif
