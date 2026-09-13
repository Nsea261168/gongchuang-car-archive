/* Drivers/Actuator: 4-channel servo PWM on TIM1/TIM2 (DM-MC02 P11 header).
 *
 * Pin / timer mapping. Servo numbers follow the wheel-leg project convention:
 *   SERVO_1  1号   PA0   TIM2_CH1
 *   SERVO_2  2号   PA2   TIM2_CH3
 *   SERVO_3  3号   PE9   TIM1_CH1
 *   SERVO_4  4号   PE13  TIM1_CH3
 *
 * Frame: 50 Hz (20 ms); pulse 500..2500 us maps 0..180 deg.
 */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

typedef enum
{
  SERVO_1 = 0,   /* 1号：PA0  / TIM2_CH1 */
  SERVO_2,       /* 2号：PA2  / TIM2_CH3 */
  SERVO_3,       /* 3号：PE9  / TIM1_CH1 */
  SERVO_4,       /* 4号：PE13 / TIM1_CH3 */
  SERVO_COUNT
} ServoId_t;

/* Reconfigures PA0/PA2/PE9/PE13 as TIM1/TIM2 PWM and centers all servos.
 * Returns 0 on success. */
uint8_t ServoPwm_Init(void);

/* Angle in degrees, clamped to 0..180 (500..2500 us). */
void ServoPwm_SetAngle(ServoId_t servo, float angle_deg);

/* Raw pulse in us, clamped to 500..2500. */
void ServoPwm_SetPulseUs(ServoId_t servo, uint16_t pulse_us);

#endif
