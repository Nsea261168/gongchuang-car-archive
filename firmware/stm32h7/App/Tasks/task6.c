#if 0 /* Retained legacy single-channel implementation for rollback. */
#include "task6.h"

#include "lcd.h"
#include "songjia_motor.h"

#include <stdio.h>
#include <string.h>

#define TASK6_QUERY_PERIOD_MS       25U
#define TASK6_CONTROL_PERIOD_MS     20U
#define TASK6_DISPLAY_PERIOD_MS    250U
#define TASK6_CONFIG_STEP_MS        60U
#define TASK6_FEEDBACK_TIMEOUT_MS  150U
#define TASK6_BOARD_TARGET_STEP       5
#define TASK6_BOARD_TARGET_LIMIT     30
#define TASK6_PID_TARGET_STEP        50
#define TASK6_PID_TARGET_LIMIT      400
#define TASK6_PID_OUTPUT_LIMIT      300
#define TASK6_REVERSE_MIN_CPS        40
#define TASK6_REVERSE_FRAME_LIMIT     3U
/* Current MG512 + 80 mm mecanum wheel wiring reports encoder A opposite to
 * the Songjia positive-speed convention.  Keep motor wiring unchanged and
 * normalize only the host-side feedback sign. */
#define TASK6_ENCODER_DIRECTION_SIGN (-1)

#define TASK6_PID_DT_S       0.02f
#define TASK6_PID_KP         0.08f
#define TASK6_PID_KI         0.10f
#define TASK6_PID_KD         0.001f
#define TASK6_PID_I_LIMIT 1000.0f
#define TASK6_FF_SLOPE       0.40f
#define TASK6_FF_OFFSET     20.0f

typedef enum
{
  TASK6_MODE_BOARD_SPEED = 0,
  TASK6_MODE_DM_PID
} Task6Mode_t;

typedef enum
{
  TASK6_SETUP_MOTOR_TYPE = 0,
  TASK6_SETUP_POLARITY,
  TASK6_SETUP_READY
} Task6Setup_t;

typedef enum
{
  TASK6_FAULT_NONE = 0,
  TASK6_FAULT_NO_LINK,
  TASK6_FAULT_REVERSE
} Task6Fault_t;

typedef struct
{
  float integral;
  float previous_error;
} Task6Pid_t;

static Task6Mode_t sMode;
static Task6Setup_t sSetup;
static Task6Fault_t sFault;
static Task6Pid_t sPid;
static uint8_t sRunning;
static uint8_t sReverseFrames;
static int16_t sBoardTargetCentiMps;
static int16_t sPidTargetCps;
static int16_t sAppliedTarget;
static int16_t sActualCps;
static int16_t sOutputPermille;
static uint32_t sSetupTick;
static uint32_t sQueryTick;
static uint32_t sControlTick;
static uint32_t sDisplayTick;
static uint32_t sFeedbackSequence;
static uint8_t sUiForceRefresh;
static uint8_t sUiCacheValid;
static char sUiMode[20];
static char sUiState[20];
static char sUiTarget[20];
static char sUiEncoder[20];
static char sUiOutput[20];
static uint16_t sUiModeColor;
static uint16_t sUiStateColor;
static uint16_t sUiTargetColor;
static uint16_t sUiEncoderColor;
static uint16_t sUiOutputColor;

static int16_t ClampI16(int16_t value, int16_t low, int16_t high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int16_t StepToward(int16_t value, int16_t target, int16_t step)
{
  if (value < target)
  {
    value = (int16_t)(value + step);
    return (value > target) ? target : value;
  }
  if (value > target)
  {
    value = (int16_t)(value - step);
    return (value < target) ? target : value;
  }
  return value;
}

static void ResetPid(void)
{
  sPid.integral = 0.0f;
  sPid.previous_error = 0.0f;
  sOutputPermille = 0;
}

static int16_t UpdatePid(int16_t target_cps, int16_t actual_cps)
{
  float error = (float)target_cps - (float)actual_cps;
  float derivative;
  float feedforward = TASK6_FF_SLOPE * (float)target_cps;
  float output;

  sPid.integral += error * TASK6_PID_DT_S;
  if (sPid.integral > TASK6_PID_I_LIMIT) sPid.integral = TASK6_PID_I_LIMIT;
  if (sPid.integral < -TASK6_PID_I_LIMIT) sPid.integral = -TASK6_PID_I_LIMIT;
  derivative = (error - sPid.previous_error) / TASK6_PID_DT_S;
  if (target_cps > 0) feedforward += TASK6_FF_OFFSET;
  if (target_cps < 0) feedforward -= TASK6_FF_OFFSET;
  sPid.previous_error = error;

  output = feedforward + TASK6_PID_KP * error +
           TASK6_PID_KI * sPid.integral + TASK6_PID_KD * derivative;
  if (output > (float)TASK6_PID_OUTPUT_LIMIT)
    output = (float)TASK6_PID_OUTPUT_LIMIT;
  if (output < (float)-TASK6_PID_OUTPUT_LIMIT)
    output = (float)-TASK6_PID_OUTPUT_LIMIT;
  return (int16_t)output;
}

/* Follow Task3's verified LCD pattern: draw the complete frame with the
 * display disabled, then refresh a whole labelled row at a time. */
static void Task6_ShowValue(uint16_t y, const char *label, const char *value,
                            uint16_t color)
{
  LCD_Fill(16U, y, 270U, (uint16_t)(y + 19U), BLACK);
  LCD_ShowString(18U, y, (const uint8_t *)label, GRAY, BLACK, 16U, 0U);
  LCD_ShowString(122U, y, (const uint8_t *)value, color, BLACK, 16U, 0U);
}

/* LCD writes are synchronous and run in the main loop.  Redraw a row only
 * when its content changes, so feedback polling cannot starve input polling. */
static void Task6_ShowValueIfChanged(uint16_t y, const char *label,
                                     const char *value, uint16_t color,
                                     char *cache, uint16_t cache_size,
                                     uint16_t *cache_color)
{
  if ((sUiForceRefresh != 0U) || (sUiCacheValid == 0U) ||
      (*cache_color != color) || (strcmp(cache, value) != 0))
  {
    Task6_ShowValue(y, label, value, color);
    (void)snprintf(cache, cache_size, "%s", value);
    *cache_color = color;
  }
}

static void Task6_DrawFrame(void)
{
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_Fill(0U, 0U, 7U, LCD_H, BRRED);
  LCD_ShowString(18U, 10U, (const uint8_t *)"SONGJIA / MG512", WHITE,
                 BLACK, 24U, 0U);
  LCD_DrawLine(18U, 38U, 268U, 38U, BRRED);
  LCD_ShowString(18U, 178U, (const uint8_t *)"UP/DN: DIRECT SPEED", GRAY,
                 BLACK, 16U, 0U);
  LCD_ShowString(18U, 198U, (const uint8_t *)"RIGHT: CHANGE MODE", GRAY,
                 BLACK, 16U, 0U);
  LCD_ShowString(18U, 220U, (const uint8_t *)"LEFT: E-STOP / BACK", YELLOW,
                 BLACK, 16U, 0U);
  LCD_WR_REG(0x29U);
}

static void Task6_DrawValues(uint32_t now_ms)
{
  char text[28];
  const char *state_text;
  uint8_t link_ok = SongjiaMotor_IsFeedbackFresh(now_ms,
                                                  TASK6_FEEDBACK_TIMEOUT_MS);
  uint16_t state_color = (sFault != TASK6_FAULT_NONE) ? RED :
                         ((link_ok != 0U) ? GREEN : YELLOW);

  Task6_ShowValueIfChanged(48U, "MODE", (sMode == TASK6_MODE_BOARD_SPEED) ?
                          "BOARD SPEED" : "DM HOST PID", CYAN,
                          sUiMode, sizeof(sUiMode), &sUiModeColor);
  if (sFault == TASK6_FAULT_NO_LINK)
  {
    state_text = "FAULT: LINK";
    state_color = RED;
  }
  else if (sFault == TASK6_FAULT_REVERSE)
  {
    state_text = "FAULT: DIR";
    state_color = RED;
  }
  else if (sSetup != TASK6_SETUP_READY)
  {
    state_text = "CONFIG...";
    state_color = YELLOW;
  }
  else if (sRunning != 0U)
  {
    state_text = (link_ok != 0U) ? "RUN / LINK OK" : "RUN / NO FB";
  }
  else if (link_ok != 0U)
  {
    state_text = "STOP / READY";
  }
  else
  {
    state_text = "STOP / NO LINK";
  }
  Task6_ShowValueIfChanged(72U, "STATE", state_text, state_color,
                          sUiState, sizeof(sUiState), &sUiStateColor);

  if (sMode == TASK6_MODE_BOARD_SPEED)
  {
    (void)snprintf(text, sizeof(text), "%+d cm/s", sBoardTargetCentiMps);
    Task6_ShowValueIfChanged(96U, "TARGET", text, WHITE,
                            sUiTarget, sizeof(sUiTarget), &sUiTargetColor);
    (void)snprintf(text, sizeof(text), "%+d cm/s", sAppliedTarget);
    Task6_ShowValueIfChanged(144U, "OUTPUT", text, WHITE,
                            sUiOutput, sizeof(sUiOutput), &sUiOutputColor);
  }
  else
  {
    (void)snprintf(text, sizeof(text), "%+d cps", sPidTargetCps);
    Task6_ShowValueIfChanged(96U, "TARGET", text, WHITE,
                            sUiTarget, sizeof(sUiTarget), &sUiTargetColor);
    (void)snprintf(text, sizeof(text), "%+d /1000", sOutputPermille);
    Task6_ShowValueIfChanged(144U, "OUTPUT", text, WHITE,
                            sUiOutput, sizeof(sUiOutput), &sUiOutputColor);
  }
  (void)snprintf(text, sizeof(text), "%+d cps", sActualCps);
  Task6_ShowValueIfChanged(120U, "ENC A", text, WHITE,
                          sUiEncoder, sizeof(sUiEncoder), &sUiEncoderColor);
  sUiCacheValid = 1U;
  sUiForceRefresh = 0U;
}

static void StopMotor(void)
{
  (void)SongjiaMotor_Stop();
  sRunning = 0U;
  sAppliedTarget = 0;
  sReverseFrames = 0U;
  ResetPid();
}

static void CheckDirection(void)
{
  int16_t target = sAppliedTarget;
  int16_t abs_actual = (sActualCps < 0) ? (int16_t)-sActualCps : sActualCps;
  if ((target != 0) && (abs_actual >= TASK6_REVERSE_MIN_CPS) &&
      (((target > 0) && (sActualCps < 0)) ||
       ((target < 0) && (sActualCps > 0))))
  {
    if (++sReverseFrames >= TASK6_REVERSE_FRAME_LIMIT)
    {
      sFault = TASK6_FAULT_REVERSE;
      StopMotor();
    }
  }
  else
  {
    sReverseFrames = 0U;
  }
}

static void Task6_ApplyTargetImmediately(uint32_t now_ms)
{
  int16_t desired_target = (sMode == TASK6_MODE_BOARD_SPEED) ?
                           sBoardTargetCentiMps : sPidTargetCps;

  if (desired_target == 0)
  {
    StopMotor();
    return;
  }

  if (sRunning == 0U)
  {
    sFault = TASK6_FAULT_NONE;
    sReverseFrames = 0U;
    ResetPid();
    sRunning = 1U;
  }
  sAppliedTarget = desired_target;
  /* Let this same scheduler pass issue the command; do not wait 20 ms. */
  sControlTick = now_ms - TASK6_CONTROL_PERIOD_MS;
}

static void HandleInput(InputEvent_t event, uint32_t now_ms)
{
  if (event == INPUT_EVENT_RIGHT)
  {
    if (sRunning == 0U)
    {
      sMode = (sMode == TASK6_MODE_BOARD_SPEED) ? TASK6_MODE_DM_PID :
                                                  TASK6_MODE_BOARD_SPEED;
      sAppliedTarget = 0;
      ResetPid();
    }
  }
  else if (event == INPUT_EVENT_UP)
  {
    if (sMode == TASK6_MODE_BOARD_SPEED)
      sBoardTargetCentiMps = ClampI16(
          (int16_t)(sBoardTargetCentiMps + TASK6_BOARD_TARGET_STEP),
          -TASK6_BOARD_TARGET_LIMIT, TASK6_BOARD_TARGET_LIMIT);
    else
      sPidTargetCps = ClampI16((int16_t)(sPidTargetCps + TASK6_PID_TARGET_STEP),
                               -TASK6_PID_TARGET_LIMIT, TASK6_PID_TARGET_LIMIT);
    if (sSetup == TASK6_SETUP_READY)
      Task6_ApplyTargetImmediately(now_ms);
  }
  else if (event == INPUT_EVENT_DOWN)
  {
    if (sMode == TASK6_MODE_BOARD_SPEED)
      sBoardTargetCentiMps = ClampI16(
          (int16_t)(sBoardTargetCentiMps - TASK6_BOARD_TARGET_STEP),
          -TASK6_BOARD_TARGET_LIMIT, TASK6_BOARD_TARGET_LIMIT);
    else
      sPidTargetCps = ClampI16((int16_t)(sPidTargetCps - TASK6_PID_TARGET_STEP),
                               -TASK6_PID_TARGET_LIMIT, TASK6_PID_TARGET_LIMIT);
    if (sSetup == TASK6_SETUP_READY)
      Task6_ApplyTargetImmediately(now_ms);
  }
  /* Center is deliberately unused in this test task. */
  if (event != INPUT_EVENT_NONE)
    sUiForceRefresh = 1U;
}

static void Task6_Enter(uint32_t now_ms)
{
  sMode = TASK6_MODE_BOARD_SPEED;
  sSetup = TASK6_SETUP_MOTOR_TYPE;
  sFault = TASK6_FAULT_NONE;
  sRunning = 0U;
  sReverseFrames = 0U;
  sBoardTargetCentiMps = 0;
  sPidTargetCps = 0;
  sAppliedTarget = 0;
  sActualCps = 0;
  sSetupTick = now_ms;
  sQueryTick = now_ms;
  sControlTick = now_ms;
  sDisplayTick = now_ms;
  sFeedbackSequence = 0U;
  sUiForceRefresh = 1U;
  sUiCacheValid = 0U;
  ResetPid();
  Task6_DrawFrame();
  Task6_DrawValues(now_ms);

  /* Keep a visible CONFIG page before touching USART1. If board-level UART
   * setup ever fails, the LCD remains useful for fault localization. */
  SongjiaMotor_Init();
  (void)SongjiaMotor_Stop();
  (void)SongjiaMotor_SetMotorType(0U); /* MG512 uses the supplied 520 profile. */
}

static void Task6_Tick(uint32_t now_ms, InputEvent_t event)
{
  const SongjiaMotorFeedback_t *feedback;
  int32_t actual_cps;
  int16_t desired_target;
  uint8_t new_feedback = 0U;

  SongjiaMotor_Process(now_ms);

  if ((sSetup == TASK6_SETUP_MOTOR_TYPE) &&
      ((uint32_t)(now_ms - sSetupTick) >= TASK6_CONFIG_STEP_MS))
  {
    (void)SongjiaMotor_SetEncoderPolarity(0U);
    sSetup = TASK6_SETUP_POLARITY;
    sSetupTick = now_ms;
  }
  else if ((sSetup == TASK6_SETUP_POLARITY) &&
           ((uint32_t)(now_ms - sSetupTick) >= TASK6_CONFIG_STEP_MS))
  {
    sSetup = TASK6_SETUP_READY;
    sQueryTick = now_ms - TASK6_QUERY_PERIOD_MS;
  }

  if ((sSetup == TASK6_SETUP_READY) &&
      ((uint32_t)(now_ms - sQueryTick) >= TASK6_QUERY_PERIOD_MS))
  {
    sQueryTick = now_ms;
    (void)SongjiaMotor_RequestEncoder20ms();
  }

  HandleInput(event, now_ms);
  feedback = SongjiaMotor_GetFeedback();
  if (feedback->sequence != sFeedbackSequence)
  {
    sFeedbackSequence = feedback->sequence;
    actual_cps = (int32_t)feedback->encoder_20ms[0] * 50 *
                 TASK6_ENCODER_DIRECTION_SIGN;
    if (actual_cps > 32767) actual_cps = 32767;
    if (actual_cps < -32768) actual_cps = -32768;
    sActualCps = (int16_t)actual_cps;
    new_feedback = 1U;
    if (sRunning != 0U)
      CheckDirection();
  }

  if ((sRunning != 0U) && (sMode == TASK6_MODE_DM_PID) &&
      (SongjiaMotor_IsFeedbackFresh(now_ms, TASK6_FEEDBACK_TIMEOUT_MS) == 0U))
  {
    sFault = TASK6_FAULT_NO_LINK;
    StopMotor();
  }

  if ((sRunning != 0U) && (sMode == TASK6_MODE_BOARD_SPEED) &&
      ((uint32_t)(now_ms - sControlTick) >= TASK6_CONTROL_PERIOD_MS))
  {
    sControlTick = now_ms;
    desired_target = sBoardTargetCentiMps;
    sAppliedTarget = StepToward(sAppliedTarget, desired_target, 1);
    (void)SongjiaMotor_SetSpeedCentiMps(sAppliedTarget, 0, 0, 0);
  }
  else if ((sRunning != 0U) && (sMode == TASK6_MODE_DM_PID) &&
           (new_feedback != 0U))
  {
    sAppliedTarget = sPidTargetCps;
    sOutputPermille = UpdatePid(sAppliedTarget, sActualCps);
    (void)SongjiaMotor_SetPwmPermille(sOutputPermille, 0, 0, 0);
  }

  if ((uint32_t)(now_ms - sDisplayTick) >= TASK6_DISPLAY_PERIOD_MS)
  {
    sDisplayTick = now_ms;
    Task6_DrawValues(now_ms);
  }
}

static void Task6_Exit(void)
{
  StopMotor();
}

const AppTask_t Task6_Definition =
{
  Task6_Enter,
  Task6_Tick,
  Task6_Exit
};
#endif

#include "mg513_task.c"
