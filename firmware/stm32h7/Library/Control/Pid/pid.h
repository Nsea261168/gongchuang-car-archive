/* Algorithm: generic PID interface. */
#ifndef PID_H
#define PID_H

#include <stdint.h>

/* Optimization switches based on the reference project's controller.c. */
typedef enum {
  PID_INTEGRAL_LIMIT = 1U << 0,
  PID_D_ON_MEASUREMENT = 1U << 1,
  PID_TRAPEZOID_I = 1U << 2,
  PID_OUTPUT_LPF = 1U << 4,
  PID_VARIABLE_I = 1U << 5,
  PID_DERIVATIVE_LPF = 1U << 6,
  PID_BLOCKED_CHECK = 1U << 7
} PidImprove_t;

typedef struct {
  float kp, ki, kd, max_out, deadband, integral_limit;
  float coef_a, coef_b, output_lpf_rc_s, derivative_lpf_rc_s;
  uint16_t improve;
} PidConfig_t;

typedef struct {
  PidConfig_t cfg;
  float measure, last_measure, error, last_error;
  float p_out, i_out, d_out, last_d_out, output, last_output;
  uint16_t blocked_count;
  uint8_t blocked;
} Pid_t;

void Pid_Init(Pid_t *pid, const PidConfig_t *cfg);
void Pid_Reset(Pid_t *pid);
float Pid_Update(Pid_t *pid, float measure, float reference, float dt_s);

#endif
