/* Application: generic boot scheduler and task-selection readiness. */
#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdint.h>
#include "bmi088_attitude.h"
#include "chassis_motion.h"
#include "../Tests/chassis_escape_test.h"

void AppScheduler_Init(void);
void AppScheduler_Run1kHz(uint32_t now_ms);
void AppScheduler_UpdateStatusLed(uint32_t now_ms);
void AppScheduler_SafeStop(void);
uint8_t AppScheduler_IsReadyForTask(uint32_t now_ms);
const BMI088_Attitude_t *AppScheduler_GetAttitude(void);

typedef enum
{
  CHASSIS_SAFETY_READY = 0,
  CHASSIS_SAFETY_POLARITY,
  CHASSIS_SAFETY_LOCALIZATION,
  CHASSIS_SAFETY_LIDAR_STALE,
  CHASSIS_SAFETY_FEEDBACK_STALE,
  CHASSIS_SAFETY_OBSTACLE
#if CHASSIS_ESCAPE_TEST_ENABLE
  ,
  CHASSIS_SAFETY_IMU
#endif
} ChassisSafetyReason_t;

typedef struct
{
  uint8_t localization_usable;
  uint8_t lidar_fresh;
  uint8_t require_feedback;
  float obstacle_distance_mm;
  float stop_distance_mm;
  uint32_t feedback_timeout_ms;
} ChassisSafetyInput_t;

ChassisSafetyReason_t AppScheduler_CheckChassisSafety(
    const ChassisSafetyInput_t *input, uint32_t now_ms);
uint8_t AppScheduler_SendSafeSpeed(const ChassisSafetyInput_t *input,
                                   uint32_t now_ms, const int16_t motor[4]);
uint8_t AppScheduler_SendSafePwm(const ChassisSafetyInput_t *input,
                                 uint32_t now_ms, const int16_t motor[4]);
void AppScheduler_RunChassisMotion(uint32_t now_ms,
                                   const ChassisSafetyInput_t *input);

#if CHASSIS_ESCAPE_TEST_ENABLE
ChassisSafetyReason_t AppScheduler_CheckChassisEscapeSafety(
    const ChassisSafetyInput_t *input, uint32_t now_ms);
void AppScheduler_RunChassisEscapeTest(uint32_t now_ms,
                                       const ChassisSafetyInput_t *input);
#endif

#endif
