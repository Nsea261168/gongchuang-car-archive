#include "app_runtime.h"

#include "main.h"
#include "app_scheduler.h"
#include "boot_ui.h"
#include "chassis_motion.h"
#include "task4.h"

#define APP_SPLASH_DURATION_MS 1000U

static uint32_t sBootTick;
static uint8_t sCalibrationPageShown;

void AppRuntime_Init(void)
{
  AppScheduler_Init();
  ChassisMotion_Init();
#if CHASSIS_ESCAPE_TEST_ENABLE
  ChassisEscapeTest_Init();
#endif
  sBootTick = HAL_GetTick();
  sCalibrationPageShown = 0U;
  BootUi_ShowSplash();
}

void AppRuntime_Tick(uint32_t now_ms)
{
  ChassisSafetyInput_t safety_input;
#if CHASSIS_ESCAPE_TEST_ENABLE
  ChassisEscapeInput_t escape_input;
  ChassisSafetyInput_t escape_safety_input;
  ChassisSafetyReason_t escape_reason;
#endif

  AppScheduler_Run1kHz(now_ms);
  Task4_FillChassisSafetyInput(&safety_input, now_ms, 1U, 150U, 250.0f);
#if CHASSIS_ESCAPE_TEST_ENABLE
  Task4_FillChassisEscapeInput(&escape_input, now_ms);
  escape_safety_input = safety_input;
  if (escape_input.obstacle_stop != 0U)
    escape_safety_input.obstacle_distance_mm = 1.0f;
  escape_reason = AppScheduler_CheckChassisEscapeSafety(&escape_safety_input,
                                                        now_ms);
  ChassisEscapeTest_NotifySafety(
      (escape_reason == CHASSIS_SAFETY_READY) ? 1U : 0U,
      (uint8_t)escape_reason);
  ChassisEscapeTest_Update(now_ms, &escape_input);
  if (ChassisEscapeTest_IsHandoffPending() != 0U)
  {
    ChassisMotion_Start(ChassisEscapeTest_GetSelectedStart(), now_ms);
    ChassisEscapeTest_AcknowledgeHandoff();
  }
  AppScheduler_RunChassisEscapeTest(now_ms, &escape_safety_input);
#endif
  AppScheduler_RunChassisMotion(now_ms, &safety_input);
  AppScheduler_UpdateStatusLed(now_ms);

  if ((sCalibrationPageShown == 0U) &&
      ((uint32_t)(now_ms - sBootTick) >= APP_SPLASH_DURATION_MS))
  {
    sCalibrationPageShown = 1U;
    BootUi_ShowGyroCalibration();
  }
}

uint8_t AppRuntime_IsReady(void)
{
  return (sCalibrationPageShown != 0U) &&
         AppScheduler_IsReadyForTask(HAL_GetTick());
}
