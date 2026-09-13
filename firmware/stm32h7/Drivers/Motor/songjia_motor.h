#ifndef SONGJIA_MOTOR_H
#define SONGJIA_MOTOR_H

#include <stdint.h>

typedef struct
{
  int16_t encoder_20ms[4];
  uint32_t received_tick_ms;
  uint32_t sequence;
  uint32_t invalid_frame_count;
} SongjiaMotorFeedback_t;

typedef struct
{
  uint8_t confirmed;
  uint8_t value;
  uint32_t last_attempt_ms;
  uint32_t ack_tick_ms;
  uint32_t attempt_count;
  uint32_t invalid_ack_count;
} SongjiaEncoderPolarityState_t;

void SongjiaMotor_Init(void);
void SongjiaMotor_Process(uint32_t now_ms);
void SongjiaMotor_StartEncoderPolarityConfig(uint8_t polarity, uint32_t now_ms);
void SongjiaMotor_CancelEncoderPolarityConfig(void);
uint8_t SongjiaMotor_IsEncoderPolarityReady(void);
uint8_t SongjiaMotor_IsEncoderPolarityFailed(void);
const SongjiaEncoderPolarityState_t *SongjiaMotor_GetEncoderPolarityState(void);
uint8_t SongjiaMotor_SetMotorType(uint8_t type);
uint8_t SongjiaMotor_SetEncoderPolarity(uint8_t polarity);
uint8_t SongjiaMotor_SetParameters(uint8_t wheel_diameter_cm,
                                   uint8_t encoder_ring_lines,
                                   uint8_t motor_gear_ratio);
uint8_t SongjiaMotor_RequestEncoder20ms(void);
uint8_t SongjiaMotor_SetSpeedCentiMps(int16_t motor_a, int16_t motor_b,
                                      int16_t motor_c, int16_t motor_d);
uint8_t SongjiaMotor_SetPwmPermille(int16_t motor_a, int16_t motor_b,
                                    int16_t motor_c, int16_t motor_d);
uint8_t SongjiaMotor_Stop(void);
const SongjiaMotorFeedback_t *SongjiaMotor_GetFeedback(void);
uint8_t SongjiaMotor_IsFeedbackFresh(uint32_t now_ms, uint32_t timeout_ms);

#endif
