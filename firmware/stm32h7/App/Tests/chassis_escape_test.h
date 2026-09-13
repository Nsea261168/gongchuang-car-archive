#ifndef CHASSIS_ESCAPE_TEST_H
#define CHASSIS_ESCAPE_TEST_H

#include <stdint.h>

/* Set to 1 only for the removable startup-escape bench test. */
#ifndef CHASSIS_ESCAPE_TEST_ENABLE
#define CHASSIS_ESCAPE_TEST_ENABLE 0U
#endif

typedef enum
{
  CHASSIS_ESCAPE_IDLE = 0,
  CHASSIS_ESCAPE_WAIT_SCAN,
  CHASSIS_ESCAPE_STRAIGHT,
  CHASSIS_ESCAPE_BRAKE,
  CHASSIS_ESCAPE_RELOCALIZE,
  CHASSIS_ESCAPE_HANDOFF,
  CHASSIS_ESCAPE_FAILED,
  CHASSIS_ESCAPE_PAUSED_SAFETY
} ChassisEscapeStage_t;

typedef struct
{
  uint8_t selected_start;
  uint8_t fixed;
  uint8_t lidar_fresh;
  uint8_t encoder_fresh;
  uint8_t obstacle_stop;
  float yaw_deg;
  uint32_t encoder_sequence;
  int16_t left_encoder_delta;
  int16_t right_encoder_delta;
} ChassisEscapeInput_t;

typedef struct
{
  uint8_t active;
  uint8_t selected_start;
  uint8_t stage;
  uint8_t attempt;
  uint8_t safety_blocked;
  uint8_t safety_reason;
  uint8_t handoff_pending;
  uint8_t calibration_ready;
  int16_t left_centi_mps;
  int16_t right_centi_mps;
  float attempt_distance_mm;
  float total_distance_mm;
  float target_yaw_deg;
} ChassisEscapeRequest_t;

void ChassisEscapeTest_Init(void);
void ChassisEscapeTest_Start(uint8_t selected_start, uint32_t now_ms);
void ChassisEscapeTest_Abort(void);
void ChassisEscapeTest_NotifySafety(uint8_t allowed, uint8_t reason);
void ChassisEscapeTest_Update(uint32_t now_ms,
                              const ChassisEscapeInput_t *input);
const ChassisEscapeRequest_t *ChassisEscapeTest_GetRequest(void);
uint8_t ChassisEscapeTest_IsHandoffPending(void);
uint8_t ChassisEscapeTest_GetSelectedStart(void);
void ChassisEscapeTest_AcknowledgeHandoff(void);

#endif
