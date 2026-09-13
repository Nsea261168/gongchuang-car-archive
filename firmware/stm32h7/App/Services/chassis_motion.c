#include "chassis_motion.h"

#include <math.h>
#include <string.h>

#define CM_PI                         3.14159265f
#define CM_FIRST_DISTANCE_MM          500.0f
#define CM_SECOND_DISTANCE_MM         500.0f
#define CM_STRAIGHT_SPEED_CENTI        10
#define CM_TURN_INNER_SPEED_CENTI       4
#define CM_TURN_OUTER_SPEED_CENTI      10
#define CM_SPEED_STEP_CENTI            10
#define CM_BRAKE_TIME_MS              100U
#define CM_FINISH_BRAKE_TIME_MS       150U
#define CM_DISTANCE_TOLERANCE_MM       30.0f
#define CM_BRAKE_ZONE_MM              100.0f
#define CM_YAW_TOLERANCE_DEG            3.0f
#define CM_STABLE_CYCLES                3U
#define CM_FIELD_MIN_MM                 0.0f
#define CM_FIELD_MAX_MM              3000.0f

static ChassisMotionRequest_t sRequest;
static ChassisMotionStage_t sStage;
static ChassisMotionStage_t sResumeStage;
static uint8_t sSelectedStart;
static uint8_t sStableCycles;
static uint32_t sStageTick;
static uint32_t sLastUpdateTick;
static float sLegStartX;
static float sLegStartY;
static float sFirstYaw;
static float sSecondYaw;
static float sTurnTargetYaw;
static float sTargetX;
static float sTargetY;
static int16_t sAppliedLeft;
static int16_t sAppliedRight;

static float Wrap180(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

static float AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ClampFloat(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int16_t StepInt16(int16_t value, int16_t target)
{
  if (value < target)
  {
    value = (int16_t)(value + CM_SPEED_STEP_CENTI);
    return (value > target) ? target : value;
  }
  if (value > target)
  {
    value = (int16_t)(value - CM_SPEED_STEP_CENTI);
    return (value < target) ? target : value;
  }
  return value;
}

static void SetDesiredSpeed(int16_t left, int16_t right)
{
  sAppliedLeft = StepInt16(sAppliedLeft, left);
  sAppliedRight = StepInt16(sAppliedRight, right);
  sRequest.left_centi_mps = sAppliedLeft;
  sRequest.right_centi_mps = sAppliedRight;
}

static void StopDesiredSpeed(void)
{
  SetDesiredSpeed(0, 0);
}

static uint8_t IsTargetInsideField(float x, float y)
{
  return ((x >= CM_FIELD_MIN_MM) && (x <= CM_FIELD_MAX_MM) &&
          (y >= CM_FIELD_MIN_MM) && (y <= CM_FIELD_MAX_MM)) ? 1U : 0U;
}

static void GetStartYaw(uint8_t start, float *yaw)
{
  *yaw = ((start == 1U) || (start == 2U)) ? 180.0f : 0.0f;
}

static void GetDirection(float yaw, float *dx, float *dy)
{
  float radians = yaw * CM_PI / 180.0f;
  *dx = sinf(radians);
  *dy = cosf(radians);
}

static void EnterStage(ChassisMotionStage_t stage, uint32_t now_ms)
{
  sStage = stage;
  sStageTick = now_ms;
  sStableCycles = 0U;
  sRequest.stage = (uint8_t)stage;
}

static void SetStraightTarget(float start_x, float start_y, float yaw,
                              float distance)
{
  float dx;
  float dy;
  GetDirection(yaw, &dx, &dy);
  sLegStartX = start_x;
  sLegStartY = start_y;
  sTargetX = start_x + dx * distance;
  sTargetY = start_y + dy * distance;
  sRequest.target_x_mm = sTargetX;
  sRequest.target_y_mm = sTargetY;
  sRequest.target_yaw_deg = yaw;
}

static float ProjectionProgress(const ChassisMotionPose_t *pose,
                                float yaw)
{
  float dx;
  float dy;
  GetDirection(yaw, &dx, &dy);
  return ((pose->x_mm - sLegStartX) * dx) +
         ((pose->y_mm - sLegStartY) * dy);
}

static uint8_t TargetReached(const ChassisMotionPose_t *pose,
                             float yaw, float distance)
{
  float progress = ProjectionProgress(pose, yaw);
  float remaining = distance - progress;
  sRequest.error_mm = remaining;
  if (remaining <= CM_DISTANCE_TOLERANCE_MM)
  {
    if (sStableCycles < 255U) sStableCycles++;
    return (sStableCycles >= CM_STABLE_CYCLES) ? 1U : 0U;
  }
  sStableCycles = 0U;
  return 0U;
}

static int16_t GetStraightSpeed(const ChassisMotionPose_t *pose,
                                float yaw, float distance)
{
  float remaining = distance - ProjectionProgress(pose, yaw);
  float speed = (float)CM_STRAIGHT_SPEED_CENTI;
  if (remaining < CM_BRAKE_ZONE_MM)
  {
    speed = (remaining / CM_BRAKE_ZONE_MM) *
            (float)CM_STRAIGHT_SPEED_CENTI;
    if (speed < 20.0f) speed = 20.0f;
  }
  if (remaining <= 0.0f) speed = 0.0f;
  return (int16_t)ClampFloat(speed, 0.0f,
                             (float)CM_STRAIGHT_SPEED_CENTI);
}

static void UpdateStraight(const ChassisMotionPose_t *pose,
                           float yaw, float distance, uint32_t now_ms)
{
  int16_t speed;
  float yaw_error = Wrap180(yaw - pose->yaw_deg);
  int16_t correction = (int16_t)ClampFloat(yaw_error * 1.5f,
                                           -20.0f, 20.0f);
  if (TargetReached(pose, yaw, distance) != 0U)
  {
    StopDesiredSpeed();
    EnterStage((sStage == CHASSIS_MOTION_STRAIGHT_1) ?
               CHASSIS_MOTION_BRAKE_1 : CHASSIS_MOTION_BRAKE_2, now_ms);
    return;
  }
  speed = GetStraightSpeed(pose, yaw, distance);
  /* Yaw error is target minus measured heading: positive steers right
     (left wheel faster), negative steers left (right wheel faster). */
  SetDesiredSpeed((int16_t)ClampFloat((float)speed + correction,
                                      0.0f,
                                      (float)CM_STRAIGHT_SPEED_CENTI),
                  (int16_t)ClampFloat((float)speed - correction,
                                      0.0f,
                                      (float)CM_STRAIGHT_SPEED_CENTI));
}

static void UpdateTurn(const ChassisMotionPose_t *pose, uint32_t now_ms)
{
  float error = Wrap180(sTurnTargetYaw - pose->yaw_deg);
  sRequest.error_yaw_deg = error;
  if (AbsFloat(error) <= CM_YAW_TOLERANCE_DEG)
  {
    if (sStableCycles < 255U) sStableCycles++;
    if (sStableCycles >= CM_STABLE_CYCLES)
    {
      StopDesiredSpeed();
      EnterStage(CHASSIS_MOTION_BRAKE_TURN, now_ms);
    }
    return;
  }
  sStableCycles = 0U;
  if (error < 0.0f)
    SetDesiredSpeed(CM_TURN_INNER_SPEED_CENTI,
                    CM_TURN_OUTER_SPEED_CENTI);
  else
    SetDesiredSpeed(CM_TURN_OUTER_SPEED_CENTI,
                    CM_TURN_INNER_SPEED_CENTI);
}

static void PrepareSecondLeg(const ChassisMotionPose_t *pose,
                             uint32_t now_ms)
{
  sLegStartX = pose->x_mm;
  sLegStartY = pose->y_mm;
  SetStraightTarget(sLegStartX, sLegStartY, sSecondYaw,
                   CM_SECOND_DISTANCE_MM);
  if (IsTargetInsideField(sTargetX, sTargetY) == 0U)
  {
    sRequest.safety_blocked = 1U;
    sRequest.safety_reason = 6U;
    EnterStage(CHASSIS_MOTION_ABORTED, now_ms);
    StopDesiredSpeed();
    return;
  }
  EnterStage(CHASSIS_MOTION_STRAIGHT_2, now_ms);
}

void ChassisMotion_Init(void)
{
  memset(&sRequest, 0, sizeof(sRequest));
  sStage = CHASSIS_MOTION_IDLE;
  sResumeStage = CHASSIS_MOTION_IDLE;
  sSelectedStart = 0U;
  sStableCycles = 0U;
  sStageTick = 0U;
  sLastUpdateTick = 0U;
  sAppliedLeft = 0;
  sAppliedRight = 0;
}

void ChassisMotion_Start(uint8_t selected_start, uint32_t now_ms)
{
  if ((selected_start < 1U) || (selected_start > 4U)) return;
  ChassisMotion_Reset();
  sSelectedStart = selected_start;
  sRequest.active = 1U;
  sRequest.selected_start = selected_start;
  EnterStage(CHASSIS_MOTION_WAIT_FIX, now_ms);
}

void ChassisMotion_Reset(void)
{
  memset(&sRequest, 0, sizeof(sRequest));
  sStage = CHASSIS_MOTION_IDLE;
  sResumeStage = CHASSIS_MOTION_IDLE;
  sStableCycles = 0U;
  sStageTick = 0U;
  sLastUpdateTick = 0U;
  sAppliedLeft = 0;
  sAppliedRight = 0;
}

void ChassisMotion_Abort(void)
{
  ChassisMotion_Reset();
  sStage = CHASSIS_MOTION_ABORTED;
  sRequest.stage = (uint8_t)sStage;
}

void ChassisMotion_NotifySafety(uint8_t allowed, uint8_t reason)
{
  if ((allowed == 0U) &&
      (sStage >= CHASSIS_MOTION_STRAIGHT_1) &&
      (sStage <= CHASSIS_MOTION_STRAIGHT_2))
  {
    sResumeStage = sStage;
    sRequest.safety_blocked = 1U;
    sRequest.safety_reason = reason;
    sStage = CHASSIS_MOTION_PAUSED_SAFETY;
    sRequest.stage = (uint8_t)sStage;
    StopDesiredSpeed();
  }
  else if ((allowed != 0U) && (sStage == CHASSIS_MOTION_PAUSED_SAFETY))
  {
    sRequest.safety_blocked = 0U;
    sRequest.safety_reason = 0U;
    sStage = sResumeStage;
    sRequest.stage = (uint8_t)sStage;
  }
}

void ChassisMotion_Update(uint32_t now_ms,
                          const ChassisMotionPose_t *pose)
{
  float start_yaw;
  if ((pose == 0) || (sRequest.active == 0U)) return;
  if ((uint32_t)(now_ms - sLastUpdateTick) < 20U) return;
  sLastUpdateTick = now_ms;
  if ((sStage == CHASSIS_MOTION_PAUSED_SAFETY) ||
      (sStage == CHASSIS_MOTION_COMPLETE) ||
      (sStage == CHASSIS_MOTION_ABORTED))
  {
    StopDesiredSpeed();
    return;
  }
  if ((sStage != CHASSIS_MOTION_WAIT_FIX) &&
      ((pose->fixed == 0U) || (pose->lidar_fresh == 0U)))
  {
    sResumeStage = sStage;
    sStage = CHASSIS_MOTION_PAUSED_SAFETY;
    sRequest.safety_blocked = 1U;
    sRequest.safety_reason = 3U;
    sRequest.stage = (uint8_t)sStage;
    StopDesiredSpeed();
    return;
  }
  if (sStage == CHASSIS_MOTION_WAIT_FIX)
  {
    StopDesiredSpeed();
    if ((pose->fixed == 0U) || (pose->lidar_fresh == 0U)) return;
    GetStartYaw(sSelectedStart, &start_yaw);
    sFirstYaw = start_yaw;
    sSecondYaw = start_yaw;
    if ((sSelectedStart == 1U) || (sSelectedStart == 4U))
      sSecondYaw = Wrap180(start_yaw - 90.0f);
    else
      sSecondYaw = Wrap180(start_yaw + 90.0f);
    sLegStartX = pose->x_mm;
    sLegStartY = pose->y_mm;
    SetStraightTarget(sLegStartX, sLegStartY, sFirstYaw,
                      CM_FIRST_DISTANCE_MM);
    if (IsTargetInsideField(sTargetX, sTargetY) == 0U)
    {
      sRequest.safety_blocked = 1U;
      sRequest.safety_reason = 6U;
      EnterStage(CHASSIS_MOTION_ABORTED, now_ms);
      return;
    }
    EnterStage(CHASSIS_MOTION_STRAIGHT_1, now_ms);
    return;
  }
  if (sStage == CHASSIS_MOTION_STRAIGHT_1)
  {
    UpdateStraight(pose, sFirstYaw, CM_FIRST_DISTANCE_MM, now_ms);
    return;
  }
  if (sStage == CHASSIS_MOTION_BRAKE_1)
  {
    StopDesiredSpeed();
    if ((uint32_t)(now_ms - sStageTick) >= CM_BRAKE_TIME_MS)
    {
      sTurnTargetYaw = sSecondYaw;
      sRequest.target_yaw_deg = sTurnTargetYaw;
      EnterStage(CHASSIS_MOTION_TURN_90, now_ms);
    }
    return;
  }
  if (sStage == CHASSIS_MOTION_TURN_90)
  {
    UpdateTurn(pose, now_ms);
    return;
  }
  if (sStage == CHASSIS_MOTION_BRAKE_TURN)
  {
    StopDesiredSpeed();
    if ((uint32_t)(now_ms - sStageTick) >= CM_BRAKE_TIME_MS)
      PrepareSecondLeg(pose, now_ms);
    return;
  }
  if (sStage == CHASSIS_MOTION_STRAIGHT_2)
  {
    UpdateStraight(pose, sSecondYaw, CM_SECOND_DISTANCE_MM, now_ms);
    return;
  }
  if (sStage == CHASSIS_MOTION_BRAKE_2)
  {
    StopDesiredSpeed();
    if ((uint32_t)(now_ms - sStageTick) >= CM_FINISH_BRAKE_TIME_MS)
    {
      EnterStage(CHASSIS_MOTION_COMPLETE, now_ms);
      sRequest.active = 0U;
    }
  }
}

const ChassisMotionRequest_t *ChassisMotion_GetRequest(void)
{
  return &sRequest;
}
