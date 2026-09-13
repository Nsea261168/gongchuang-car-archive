#include "task6.h"
#include "lcd.h"
#include "songjia_motor.h"
#include <stdio.h>
#include <string.h>

#define MG513_CONFIG_STEP_MS       100U
#define MG513_QUERY_PERIOD_MS       50U
#define MG513_DISPLAY_PERIOD_MS    250U
#define MG513_FEEDBACK_TIMEOUT_MS  150U
#define MG513_SPEED_STEP_CMPS        2
#define MG513_SPEED_LIMIT_CMPS      17

typedef enum { MG513_CONFIG_MOTOR_TYPE = 0, MG513_CONFIG_ZERO_SPEED, MG513_CONFIG_READY } Mg513ConfigState_t;
typedef enum { MG513_MODE_SPEED = 0, MG513_MODE_ENCODER } Mg513Mode_t;
static Mg513ConfigState_t sConfigState;
static Mg513Mode_t sMode;
static int16_t sTargetCentiMps;
static int16_t sEncoderDelta[4];
static uint8_t sRunning, sCommandPending, sUiForceRefresh, sUiCacheValid;
static uint32_t sConfigTick, sQueryTick, sDisplayTick, sLastTxTick, sFeedbackSequence;
static char sUiCache[5][32];
static uint16_t sUiCacheColor[5];

static int16_t Mg513_ClampI16(int16_t value, int16_t low, int16_t high)
{ return value < low ? low : (value > high ? high : value); }
static void Mg513_RecordTx(uint8_t accepted, uint32_t now_ms)
{ if (accepted != 0U) sLastTxTick = now_ms; }
static void Mg513_Stop(uint32_t now_ms)
{ Mg513_RecordTx(SongjiaMotor_SetSpeedCentiMps(0, 0, 0, 0), now_ms); sRunning = 0U; sCommandPending = 0U; }
static void Mg513_SendOffsetProfile(uint32_t now_ms)
{
  int16_t a = sTargetCentiMps;
  int16_t delta = (a > 0) ? 1 : -1;
  Mg513_RecordTx(SongjiaMotor_SetSpeedCentiMps(a, (int16_t)(a + delta),
                                               (int16_t)(a + 2 * delta),
                                               (int16_t)(a + 3 * delta)), now_ms);
}

static void Mg513_ShowLine(uint16_t y, const char *text, uint16_t color, uint8_t index)
{
  if ((sUiForceRefresh == 0U) && (sUiCacheValid != 0U) && (sUiCacheColor[index] == color) && (strcmp(sUiCache[index], text) == 0)) return;
  LCD_Fill(16U, y, 270U, (uint16_t)(y + 19U), BLACK);
  LCD_ShowString(18U, y, (const uint8_t *)text, color, BLACK, 16U, 0U);
  (void)snprintf(sUiCache[index], sizeof(sUiCache[index]), "%s", text);
  sUiCacheColor[index] = color;
}
static void Mg513_DrawFrame(void)
{
  LCD_WR_REG(0x28U); LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK); LCD_Fill(0U, 0U, 7U, LCD_H, BRRED);
  LCD_ShowString(18U, 7U, (const uint8_t *)"MG513 SONGJIA", WHITE, BLACK, 24U, 0U); LCD_DrawLine(18U, 35U, 268U, 35U, BRRED);
  LCD_ShowString(18U, 196U, (const uint8_t *)"UP/DN: SPEED", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(18U, 216U, (const uint8_t *)"RIGHT:MODE LEFT:BACK", YELLOW, BLACK, 16U, 0U); LCD_WR_REG(0x29U);
}
static void Mg513_DrawValues(uint32_t now_ms)
{
  char text[32];
  uint8_t tx_recent = ((uint32_t)(now_ms - sLastTxTick) <= MG513_FEEDBACK_TIMEOUT_MS);
  if (sMode == MG513_MODE_SPEED)
  {
    Mg513_ShowLine(44U, "MODE: OFFICIAL $Car PID", CYAN, 0U);
    (void)snprintf(text, sizeof(text), "TARGET: %+d.%02d m/s", sTargetCentiMps / 100, (sTargetCentiMps < 0 ? -sTargetCentiMps : sTargetCentiMps) % 100); Mg513_ShowLine(68U, text, WHITE, 1U);
    Mg513_ShowLine(96U, "A/B/C/D: OFFSET PROFILE", WHITE, 2U);
    Mg513_ShowLine(120U, "RANGE: -0.20..+0.20", WHITE, 3U);
    (void)snprintf(text, sizeof(text), "UART TX:%s", tx_recent ? "OK" : "WAIT"); Mg513_ShowLine(148U, text, tx_recent ? GREEN : YELLOW, 4U);
  }
  else
  {
    Mg513_ShowLine(44U, "MODE: ENCODER MONITOR", CYAN, 0U);
    (void)snprintf(text, sizeof(text), "ENC A:%+d B:%+d", sEncoderDelta[0], sEncoderDelta[1]); Mg513_ShowLine(68U, text, WHITE, 1U);
    (void)snprintf(text, sizeof(text), "ENC C:%+d D:%+d", sEncoderDelta[2], sEncoderDelta[3]); Mg513_ShowLine(96U, text, WHITE, 2U);
    Mg513_ShowLine(120U, "TURN WHEEL BY HAND", WHITE, 3U);
    (void)snprintf(text, sizeof(text), "ENC RX:%s", SongjiaMotor_IsFeedbackFresh(now_ms, MG513_FEEDBACK_TIMEOUT_MS) ? "OK" : "LOST");
    Mg513_ShowLine(148U, text, SongjiaMotor_IsFeedbackFresh(now_ms, MG513_FEEDBACK_TIMEOUT_MS) ? GREEN : YELLOW, 4U);
  }
  LCD_Fill(16U, 172U, 270U, 191U, BLACK);
  LCD_ShowString(18U, 172U, (const uint8_t *)(sConfigState != MG513_CONFIG_READY ? "STATE: CONFIG" : (sRunning ? "STATE: RUN" : "STATE: STOP")), sRunning ? GREEN : YELLOW, BLACK, 16U, 0U);
  sUiCacheValid = 1U; sUiForceRefresh = 0U;
}
static void Mg513_HandleInput(InputEvent_t event, uint32_t now_ms)
{
  if (event == INPUT_EVENT_RIGHT)
  {
    Mg513_Stop(now_ms);
    sMode = (sMode == MG513_MODE_SPEED) ? MG513_MODE_ENCODER : MG513_MODE_SPEED;
    sQueryTick = now_ms - MG513_QUERY_PERIOD_MS;
  }
  else if ((sMode == MG513_MODE_SPEED) && ((event == INPUT_EVENT_UP) || (event == INPUT_EVENT_DOWN)))
  {
    int16_t step = (event == INPUT_EVENT_UP) ? MG513_SPEED_STEP_CMPS : -MG513_SPEED_STEP_CMPS;
    sTargetCentiMps = Mg513_ClampI16((int16_t)(sTargetCentiMps + step), -MG513_SPEED_LIMIT_CMPS, MG513_SPEED_LIMIT_CMPS);
    if (sTargetCentiMps == 0) Mg513_Stop(now_ms); else sCommandPending = 1U;
  }
  if (event != INPUT_EVENT_NONE) sUiForceRefresh = 1U;
}
static void Mg513_Enter(uint32_t now_ms)
{
  sConfigState = MG513_CONFIG_MOTOR_TYPE; sMode = MG513_MODE_SPEED; sTargetCentiMps = 0; sRunning = sCommandPending = 0U;
  sUiForceRefresh = 1U; sUiCacheValid = 0U; sConfigTick = sQueryTick = sDisplayTick = now_ms; sLastTxTick = sFeedbackSequence = 0U;
  memset(sEncoderDelta, 0, sizeof(sEncoderDelta));
  Mg513_DrawFrame(); Mg513_Stop(now_ms);
  SongjiaMotor_StartEncoderPolarityConfig(1U, now_ms);
  Mg513_DrawValues(now_ms);
}
static void Mg513_Tick(uint32_t now_ms, InputEvent_t event)
{
  const SongjiaMotorFeedback_t *feedback;
  uint8_t index;
  if (SongjiaMotor_IsEncoderPolarityReady() == 0U)
  {
    if ((sUiForceRefresh != 0U) || ((uint32_t)(now_ms - sDisplayTick) >= MG513_DISPLAY_PERIOD_MS))
    {
      const SongjiaEncoderPolarityState_t *polarity =
          SongjiaMotor_GetEncoderPolarityState();
      char text[32];
      sDisplayTick = now_ms;
      Mg513_ShowLine(44U, "ENC POLARITY CONFIG", YELLOW, 0U);
      Mg513_ShowLine(68U, "WAITING ACK", YELLOW, 1U);
      (void)snprintf(text, sizeof(text), "RETRY: %lu",
                     (unsigned long)polarity->attempt_count);
      Mg513_ShowLine(96U, text, WHITE, 2U);
    }
    return;
  }
  if (sConfigState == MG513_CONFIG_MOTOR_TYPE)
  {
    Mg513_RecordTx(SongjiaMotor_SetMotorType(2U), now_ms);
    Mg513_Stop(now_ms);
    sConfigState = MG513_CONFIG_ZERO_SPEED;
    sConfigTick = now_ms;
  }
  else if ((sConfigState == MG513_CONFIG_ZERO_SPEED) && ((uint32_t)(now_ms - sConfigTick) >= MG513_CONFIG_STEP_MS))
  { sConfigState = MG513_CONFIG_READY; }
  Mg513_HandleInput(event, now_ms);
  if ((sMode == MG513_MODE_ENCODER) && (sConfigState == MG513_CONFIG_READY) && ((uint32_t)(now_ms - sQueryTick) >= MG513_QUERY_PERIOD_MS))
  { sQueryTick = now_ms; Mg513_RecordTx(SongjiaMotor_RequestEncoder20ms(), now_ms); }
  if (sMode == MG513_MODE_ENCODER)
  {
    feedback = SongjiaMotor_GetFeedback();
    if (feedback->sequence != sFeedbackSequence)
    { sFeedbackSequence = feedback->sequence; for (index = 0U; index < 4U; index++) sEncoderDelta[index] = feedback->encoder_20ms[index]; }
  }
  if ((sMode == MG513_MODE_SPEED) && (sCommandPending != 0U) && (sConfigState == MG513_CONFIG_READY))
  { Mg513_SendOffsetProfile(now_ms); sRunning = 1U; sCommandPending = 0U; }
  if ((sUiForceRefresh != 0U) || ((uint32_t)(now_ms - sDisplayTick) >= MG513_DISPLAY_PERIOD_MS))
  { sDisplayTick = now_ms; Mg513_DrawValues(now_ms); }
}
static void Mg513_Exit(void)
{
  Mg513_Stop(HAL_GetTick());
  SongjiaMotor_CancelEncoderPolarityConfig();
}
const AppTask_t Mg513_Definition = { Mg513_Enter, Mg513_Tick, Mg513_Exit };
