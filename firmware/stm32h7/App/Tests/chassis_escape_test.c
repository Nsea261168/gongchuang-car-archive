#include "chassis_escape_test.h"

#include <math.h>
#include <string.h>

#if CHASSIS_ESCAPE_TEST_ENABLE

#define CET_WAIT_SCAN_MS             1000U
#define CET_ESCAPE_BRAKE_MS           150U
#define CET_RELOCALIZE_MS             300U
#define CET_ESCAPE_DISTANCE_MM       250.0f
#define CET_MAX_ATTEMPTS                3U
#define CET_MAX_TOTAL_DISTANCE_MM     750.0f
#define CET_ESCAPE_SPEED_CENTI           5
#define CET_SPEED_STEP_CENTI             5
#define CET_YAW_KP                    0.20f
#define CET_YAW_CORRECTION_MAX           2
#define CET_REASON_CAL_REQUIRED           7U
#define CET_REASON_ESCAPE_LIMIT           8U

/* Fill these values from the 500 mm bench calibration before enabling the test. */
#define CET_LEFT_MM_PER_COUNT          0.0f
#define CET_RIGHT_MM_PER_COUNT         0.0f
#define CET_LEFT_ENCODER_SIGN             1
#define CET_RIGHT_ENCODER_SIGN            1

static ChassisEscapeRequest_t sRequest;
static ChassisEscapeStage_t sStage;
static ChassisEscapeStage_t sResumeStage;
static uint32_t sStageTick;
static uint32_t sLastUpdateTick;
static uint32_t sLastEncoderSequence;
static uint8_t sEncoderBaselineSet;
static float sAttemptDistance;
static float sTotalDistance;
static float sTargetYaw;
static float sHeadingReferenceYaw;
static uint8_t sHeadingReferenceSet;
static int16_t sAppliedLeft;
static int16_t sAppliedRight;

static float Wrap180(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
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
    value = (int16_t)(value + CET_SPEED_STEP_CENTI);
    return (value > target) ? target : value;
  }
  if (value > target)
  {
    value = (int16_t)(value - CET_SPEED_STEP_CENTI);
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

static void EnterStage(ChassisEscapeStage_t stage, uint32_t now_ms)
{
  sStage = stage;
  sStageTick = now_ms;
  sRequest.stage = (uint8_t)stage;
}

static uint8_t IsCalibrationReady(void)
{
  return ((CET_LEFT_MM_PER_COUNT > 0.0f) &&
          (CET_RIGHT_MM_PER_COUNT > 0.0f)) ? 1U : 0U;
}

static void AccumulateEncoder(const ChassisEscapeInput_t *input)
{
  float left_mm;
  float right_mm;
  float delta_mm;

  if ((input->encoder_fresh == 0U) ||
      (input->encoder_sequence == sLastEncoderSequence)) return;
  if (sEncoderBaselineSet == 0U)
  {
    sLastEncoderSequence = input->encoder_sequence;
    sEncoderBaselineSet = 1U;
    return;
  }
  sLastEncoderSequence = input->encoder_sequence;
  left_mm = (float)input->left_encoder_delta *
            CET_LEFT_MM_PER_COUNT * (float)CET_LEFT_ENCODER_SIGN;
  right_mm = (float)input->right_encoder_delta *
             CET_RIGHT_MM_PER_COUNT * (float)CET_RIGHT_ENCODER_SIGN;
  delta_mm = 0.5f * (left_mm + right_mm);
  if ((delta_mm > 0.0f) && (delta_mm < 100.0f))
  {
    sAttemptDistance += delta_mm;
    sTotalDistance += delta_mm;
    sRequest.attempt_distance_mm = sAttemptDistance;
    sRequest.total_distance_mm = sTotalDistance;
  }
}

static void SetHandoff(void)
{
  StopDesiredSpeed();
  sStage = CHASSIS_ESCAPE_HANDOFF;
  sRequest.stage = (uint8_t)sStage;
  sRequest.handoff_pending = 1U;
}

void ChassisEscapeTest_Init(void)
{
  memset(&sRequest, 0, sizeof(sRequest));
  sStage = CHASSIS_ESCAPE_IDLE;
  sResumeStage = CHASSIS_ESCAPE_IDLE;
  sStageTick = 0U;
  sLastUpdateTick = 0U;
  sLastEncoderSequence = 0U;
  sEncoderBaselineSet = 0U;
  sAttemptDistance = 0.0f;
  sTotalDistance = 0.0f;
  sTargetYaw = 0.0f;
  sHeadingReferenceYaw = 0.0f;
  sHeadingReferenceSet = 0U;
  sAppliedLeft = 0;
  sAppliedRight = 0;
}

void ChassisEscapeTest_Start(uint8_t selected_start, uint32_t now_ms)
{
  if ((selected_start < 1U) || (selected_start > 4U)) return;
  ChassisEscapeTest_Init();
  sRequest.active = 1U;
  sRequest.selected_start = selected_start;
  sRequest.calibration_ready = IsCalibrationReady();
  sTargetYaw = ((selected_start == 1U) || (selected_start == 2U)) ?
               180.0f : 0.0f;
  sRequest.target_yaw_deg = sTargetYaw;
  EnterStage(CHASSIS_ESCAPE_WAIT_SCAN, now_ms);
}

void ChassisEscapeTest_Abort(void)
{
  ChassisEscapeTest_Init();
}

void ChassisEscapeTest_NotifySafety(uint8_t allowed, uint8_t reason)
{
  if ((allowed == 0U) && (sStage == CHASSIS_ESCAPE_STRAIGHT))
  {
    sResumeStage = sStage;
    sRequest.safety_blocked = 1U;
    sRequest.safety_reason = reason;
    EnterStage(CHASSIS_ESCAPE_PAUSED_SAFETY, sStageTick);
    StopDesiredSpeed();
  }
  else if ((allowed != 0U) && (sStage == CHASSIS_ESCAPE_PAUSED_SAFETY))
  {
    sRequest.safety_blocked = 0U;
    sRequest.safety_reason = 0U;
    EnterStage(sResumeStage, sStageTick);
  }
}

void ChassisEscapeTest_Update(uint32_t now_ms,
                              const ChassisEscapeInput_t *input)
{
  float yaw_error;
  int16_t correction;

  if ((input == 0) || (sRequest.active == 0U)) return;
  if ((uint32_t)(now_ms - sLastUpdateTick) < 20U) return;
  sLastUpdateTick = now_ms;

  if ((input->fixed != 0U) &&
      (sStage != CHASSIS_ESCAPE_FAILED) &&
      (sStage != CHASSIS_ESCAPE_HANDOFF))
  {
    SetHandoff();
    return;
  }

  if ((sStage == CHASSIS_ESCAPE_PAUSED_SAFETY) ||
      (sStage == CHASSIS_ESCAPE_FAILED) ||
      (sStage == CHASSIS_ESCAPE_HANDOFF))
  {
    StopDesiredSpeed();
    return;
  }

  if (sStage == CHASSIS_ESCAPE_WAIT_SCAN)
  {
    StopDesiredSpeed();
    if (sHeadingReferenceSet == 0U)
    {
      sHeadingReferenceYaw = input->yaw_deg;
      sHeadingReferenceSet = 1U;
    }
    if ((uint32_t)(now_ms - sStageTick) < CET_WAIT_SCAN_MS) return;
    if (IsCalibrationReady() == 0U)
    {
      sRequest.calibration_ready = 0U;
      sRequest.safety_reason = CET_REASON_CAL_REQUIRED;
      EnterStage(CHASSIS_ESCAPE_FAILED, now_ms);
      return;
    }
    if ((input->lidar_fresh == 0U) || (input->encoder_fresh == 0U) ||
        (input->obstacle_stop != 0U)) return;
    if ((sRequest.attempt >= CET_MAX_ATTEMPTS) ||
        (sTotalDistance >= CET_MAX_TOTAL_DISTANCE_MM))
    {
      sRequest.safety_reason = CET_REASON_ESCAPE_LIMIT;
      EnterStage(CHASSIS_ESCAPE_FAILED, now_ms);
      return;
    }
    sRequest.attempt++;
    sAttemptDistance = 0.0f;
    sRequest.attempt_distance_mm = 0.0f;
    sEncoderBaselineSet = 0U;
    EnterStage(CHASSIS_ESCAPE_STRAIGHT, now_ms);
    return;
  }

  if (sStage == CHASSIS_ESCAPE_STRAIGHT)
  {
    AccumulateEncoder(input);
    if (sAttemptDistance >= CET_ESCAPE_DISTANCE_MM)
    {
      StopDesiredSpeed();
      EnterStage(CHASSIS_ESCAPE_BRAKE, now_ms);
      return;
    }
    yaw_error = Wrap180(sHeadingReferenceYaw - input->yaw_deg);
    correction = (int16_t)ClampFloat(yaw_error * CET_YAW_KP,
                                     -(float)CET_YAW_CORRECTION_MAX,
                                     (float)CET_YAW_CORRECTION_MAX);
    SetDesiredSpeed((int16_t)(CET_ESCAPE_SPEED_CENTI + correction),
                    (int16_t)(CET_ESCAPE_SPEED_CENTI - correction));
    return;
  }

  if (sStage == CHASSIS_ESCAPE_BRAKE)
  {
    StopDesiredSpeed();
    if ((uint32_t)(now_ms - sStageTick) < CET_ESCAPE_BRAKE_MS) return;
    if ((sRequest.attempt >= CET_MAX_ATTEMPTS) ||
        (sTotalDistance >= CET_MAX_TOTAL_DISTANCE_MM))
    {
      sRequest.safety_reason = CET_REASON_ESCAPE_LIMIT;
      EnterStage(CHASSIS_ESCAPE_FAILED, now_ms);
      return;
    }
    EnterStage(CHASSIS_ESCAPE_RELOCALIZE, now_ms);
    return;
  }

  if (sStage == CHASSIS_ESCAPE_RELOCALIZE)
  {
    StopDesiredSpeed();
    if ((uint32_t)(now_ms - sStageTick) < CET_RELOCALIZE_MS) return;
    EnterStage(CHASSIS_ESCAPE_WAIT_SCAN, now_ms);
  }
}

const ChassisEscapeRequest_t *ChassisEscapeTest_GetRequest(void)
{
  return &sRequest;
}

uint8_t ChassisEscapeTest_IsHandoffPending(void)
{
  return sRequest.handoff_pending;
}

uint8_t ChassisEscapeTest_GetSelectedStart(void)
{
  return sRequest.selected_start;
}

void ChassisEscapeTest_AcknowledgeHandoff(void)
{
  ChassisEscapeTest_Init();
}

#else

void ChassisEscapeTest_Init(void) {}
void ChassisEscapeTest_Start(uint8_t selected_start, uint32_t now_ms)
{
  (void)selected_start;
  (void)now_ms;
}
void ChassisEscapeTest_Abort(void) {}
void ChassisEscapeTest_NotifySafety(uint8_t allowed, uint8_t reason)
{
  (void)allowed;
  (void)reason;
}
void ChassisEscapeTest_Update(uint32_t now_ms,
                              const ChassisEscapeInput_t *input)
{
  (void)now_ms;
  (void)input;
}
static const ChassisEscapeRequest_t sDisabledRequest = {0};
const ChassisEscapeRequest_t *ChassisEscapeTest_GetRequest(void)
{
  return &sDisabledRequest;
}
uint8_t ChassisEscapeTest_IsHandoffPending(void) { return 0U; }
uint8_t ChassisEscapeTest_GetSelectedStart(void) { return 0U; }
void ChassisEscapeTest_AcknowledgeHandoff(void) {}

#endif
