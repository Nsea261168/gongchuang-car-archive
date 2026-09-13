/* Algorithm: championship PID port. */
#include "pid.h"

static float Limit(float x, float l) { return x > l ? l : (x < -l ? -l : x); }
static float Abs(float x) { return x < 0.0f ? -x : x; }

void Pid_Init(Pid_t *pid, const PidConfig_t *cfg)
{ *pid = (Pid_t){0}; pid->cfg = *cfg; }
void Pid_Reset(Pid_t *pid)
{ PidConfig_t cfg = pid->cfg; *pid = (Pid_t){0}; pid->cfg = cfg; }

/* 按参考工程 controller.c 的计算顺序实现；dt 由 1 kHz 调度传入。 */
float Pid_Update(Pid_t *pid, float measure, float reference, float dt_s)
{
  float i_term, raw_d, raw_output, alpha;
  if (dt_s < 0.000001f) return pid->output;
  pid->measure = measure; pid->error = reference - measure;
  if (Abs(pid->error) <= pid->cfg.deadband) { Pid_Reset(pid); return 0.0f; }
  pid->p_out = pid->cfg.kp * pid->error;
  i_term = pid->cfg.ki * pid->error * dt_s;
  if (pid->cfg.improve & PID_TRAPEZOID_I)
    i_term = pid->cfg.ki * 0.5f * (pid->error + pid->last_error) * dt_s;
  if ((pid->cfg.improve & PID_VARIABLE_I) && pid->error * pid->i_out > 0.0f && pid->cfg.coef_a > 0.0f) {
    float e = Abs(pid->error);
    if (e > pid->cfg.coef_b + pid->cfg.coef_a) i_term = 0.0f;
    else if (e > pid->cfg.coef_b) i_term *= (pid->cfg.coef_a - e + pid->cfg.coef_b) / pid->cfg.coef_a;
  }
  raw_d = (pid->cfg.improve & PID_D_ON_MEASUREMENT) ?
          pid->cfg.kd * (pid->last_measure - measure) / dt_s :
          pid->cfg.kd * (pid->error - pid->last_error) / dt_s;
  if ((pid->cfg.improve & PID_DERIVATIVE_LPF) && pid->cfg.derivative_lpf_rc_s > 0.0f) {
    alpha = dt_s / (pid->cfg.derivative_lpf_rc_s + dt_s);
    raw_d = alpha * raw_d + (1.0f - alpha) * pid->last_d_out;
  }
  if (pid->cfg.improve & PID_INTEGRAL_LIMIT) {
    float next_i = Limit(pid->i_out + i_term, pid->cfg.integral_limit);
    if (Abs(pid->p_out + next_i + raw_d) <= pid->cfg.max_out || pid->error * pid->i_out < 0.0f) pid->i_out = next_i;
  } else pid->i_out += i_term;
  raw_output = Limit(pid->p_out + pid->i_out + raw_d, pid->cfg.max_out);
  if ((pid->cfg.improve & PID_OUTPUT_LPF) && pid->cfg.output_lpf_rc_s > 0.0f) {
    alpha = dt_s / (pid->cfg.output_lpf_rc_s + dt_s);
    pid->output = alpha * raw_output + (1.0f - alpha) * pid->last_output;
  } else pid->output = raw_output;
  if ((pid->cfg.improve & PID_BLOCKED_CHECK) && Abs(reference) > 0.0001f && Abs(pid->output) > pid->cfg.max_out * 0.8f && Abs(pid->error) > Abs(reference) * 0.9f) { if (++pid->blocked_count > 500U) pid->blocked = 1U; } else pid->blocked_count = 0U;
  pid->last_measure = measure; pid->last_error = pid->error; pid->last_d_out = raw_d; pid->last_output = pid->output;
  return pid->output;
}
