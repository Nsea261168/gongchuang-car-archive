#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

typedef enum
{
  CHASSIS_MOTION_IDLE = 0,
  CHASSIS_MOTION_WAIT_FIX,
  CHASSIS_MOTION_STRAIGHT_1,
  CHASSIS_MOTION_BRAKE_1,
  CHASSIS_MOTION_TURN_90,
  CHASSIS_MOTION_BRAKE_TURN,
  CHASSIS_MOTION_STRAIGHT_2,
  CHASSIS_MOTION_BRAKE_2,
  CHASSIS_MOTION_COMPLETE,
  CHASSIS_MOTION_PAUSED_SAFETY,
  CHASSIS_MOTION_ABORTED
} ChassisMotionStage_t;

typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
  uint8_t fixed;
  uint8_t lidar_fresh;
} ChassisMotionPose_t;

typedef struct
{
  uint8_t active;
  uint8_t selected_start;
  uint8_t stage;
  uint8_t safety_blocked;
  uint8_t safety_reason;
  int16_t left_centi_mps;
  int16_t right_centi_mps;
  float target_x_mm;
  float target_y_mm;
  float target_yaw_deg;
  float error_mm;
  float error_yaw_deg;
} ChassisMotionRequest_t;

void ChassisMotion_Init(void);
void ChassisMotion_Start(uint8_t selected_start, uint32_t now_ms);
void ChassisMotion_Reset(void);
void ChassisMotion_Abort(void);
void ChassisMotion_Update(uint32_t now_ms,
                          const ChassisMotionPose_t *pose);
void ChassisMotion_NotifySafety(uint8_t allowed, uint8_t reason);
const ChassisMotionRequest_t *ChassisMotion_GetRequest(void);

#endif
