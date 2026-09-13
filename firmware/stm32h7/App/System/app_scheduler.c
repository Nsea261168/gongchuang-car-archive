/* Application: boot scheduler.
 *
 * Boot flow:
 *   1. Startup delay (1 s, red solid): let power settle, no IMU reads.
 *   2. BMI088 gyro bias calibration (1000 samples, red blink).
 *   3. Ready: the key-based task selector takes over the LED.
 *
 * Motors: after the 1 s power-settle delay, ENABLE is retried at low rate
 * until each QD4310 feedback frame confirms its enabled flag. No motion
 * command is issued by the boot scheduler.
 */
#include "app_scheduler.h"

#include "main.h"
#include "fdcan.h"
#include "bsp_fdcan.h"
#include "QD4310.h"
#include "ws2812.h"
#include "bmi088_attitude.h"
#include "songjia_motor.h"
#include "ld06.h"

/* CAN1 = Motor_0, CAN2 = Motor_1. */
QD4310_t Motor_0 = {.id = 0, .hcan = &hfdcan1};
QD4310_t Motor_1 = {.id = 0, .hcan = &hfdcan2};

#define APP_STARTUP_DELAY_MS   1000U
#define APP_CONTROL_PERIOD_MS     1U
#define APP_MOTOR_ENABLE_RETRY_MS 200U
#define APP_MOTION_PERIOD_MS      20U
#define APP_FEEDBACK_QUERY_MS     50U

static BMI088_Attitude_t sImu;
static uint32_t sControlTick;
static uint32_t sLedTick;
static uint32_t sStartupDelayTick;
static uint32_t sMotorEnableTick;
static uint8_t sApplicationFault;
static uint8_t sLidarInitFailed;
static uint32_t sSafetyStopTick;
static uint32_t sMotionTick;
static uint32_t sFeedbackQueryTick;
#if CHASSIS_ESCAPE_TEST_ENABLE
static uint32_t sEscapeTestTick;
#endif

static void SendRateLimitedSafetyStop(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - sSafetyStopTick) >= 50U)
  {
    sSafetyStopTick = now_ms;
    (void)SongjiaMotor_Stop();
  }
}

void AppScheduler_SafeStop(void)
{
  (void)SongjiaMotor_Stop();
  QD4310_SetCurrent(&Motor_0, 0.0f);
  QD4310_SetCurrent(&Motor_1, 0.0f);
  QD4310_Disable(&Motor_0);
  QD4310_Disable(&Motor_1);
}

uint8_t AppScheduler_IsReadyForTask(uint32_t now_ms)
{
  if (sApplicationFault != 0U)
    return 0U;
  if ((uint32_t)(now_ms - sStartupDelayTick) < APP_STARTUP_DELAY_MS)
    return 0U;
  return (sImu.bias_ready != 0U) ? 1U : 0U;
}

void AppScheduler_Init(void)
{
  bsp_can_init();
  SongjiaMotor_Init();
  sLidarInitFailed = LD06_Init();

  sApplicationFault =
      (BMI088_Attitude_Init(&sImu) != BMI088_ATTITUDE_OK) ? 1U : 0U;
  sStartupDelayTick = HAL_GetTick();
  sControlTick = sStartupDelayTick;
  sLedTick = sStartupDelayTick;
  sMotorEnableTick = sStartupDelayTick;
  sSafetyStopTick = sStartupDelayTick - 50U;
  sMotionTick = sStartupDelayTick - APP_MOTION_PERIOD_MS;
  sFeedbackQueryTick = sStartupDelayTick - APP_FEEDBACK_QUERY_MS;
#if CHASSIS_ESCAPE_TEST_ENABLE
  sEscapeTestTick = sStartupDelayTick - APP_MOTION_PERIOD_MS;
#endif
  WS2812_Ctrl(0U, 1U, 0U);
}

void AppScheduler_Run1kHz(uint32_t now_ms)
{
  uint32_t elapsed_ms;
  SongjiaMotor_Process(now_ms);
  if (sLidarInitFailed == 0U) LD06_Process();
  elapsed_ms = (uint32_t)(now_ms - sControlTick);
  if (elapsed_ms < APP_CONTROL_PERIOD_MS)
    return;
  sControlTick = now_ms;

  /* The motor controller may finish booting after the MCU and miss a command
   * sent immediately after FDCAN initialization. Wait for power to settle,
   * then retry ENABLE only for motors whose feedback has not confirmed it. */
  if (((uint32_t)(now_ms - sStartupDelayTick) >= APP_STARTUP_DELAY_MS) &&
      ((uint32_t)(now_ms - sMotorEnableTick) >= APP_MOTOR_ENABLE_RETRY_MS) &&
      ((!Motor_0.enabled) || (!Motor_1.enabled)))
  {
    sMotorEnableTick = now_ms;
    if (!Motor_0.enabled) QD4310_Enable(&Motor_0);
    if (!Motor_1.enabled) QD4310_Enable(&Motor_1);
  }

  if ((uint32_t)(now_ms - sStartupDelayTick) >= APP_STARTUP_DELAY_MS)
  {
    if (BMI088_Attitude_Update(&sImu, (float)elapsed_ms * 0.001f) !=
        BMI088_ATTITUDE_OK)
      sApplicationFault = 1U;
  }
}

const BMI088_Attitude_t *AppScheduler_GetAttitude(void)
{
  return &sImu;
}

ChassisSafetyReason_t AppScheduler_CheckChassisSafety(
    const ChassisSafetyInput_t *input, uint32_t now_ms)
{
  if (SongjiaMotor_IsEncoderPolarityReady() == 0U)
    return CHASSIS_SAFETY_POLARITY;
  if ((input == 0) || (input->localization_usable == 0U))
    return CHASSIS_SAFETY_LOCALIZATION;
  if (input->lidar_fresh == 0U)
    return CHASSIS_SAFETY_LIDAR_STALE;
  if ((input->require_feedback != 0U) &&
      (SongjiaMotor_IsFeedbackFresh(now_ms, input->feedback_timeout_ms) == 0U))
    return CHASSIS_SAFETY_FEEDBACK_STALE;
  if ((input->obstacle_distance_mm > 0.0f) &&
      (input->obstacle_distance_mm <= input->stop_distance_mm))
    return CHASSIS_SAFETY_OBSTACLE;
  return CHASSIS_SAFETY_READY;
}

#if CHASSIS_ESCAPE_TEST_ENABLE
ChassisSafetyReason_t AppScheduler_CheckChassisEscapeSafety(
    const ChassisSafetyInput_t *input, uint32_t now_ms)
{
  if (SongjiaMotor_IsEncoderPolarityReady() == 0U)
    return CHASSIS_SAFETY_POLARITY;
  if ((sApplicationFault != 0U) || (sImu.bias_ready == 0U))
    return CHASSIS_SAFETY_IMU;
  if ((input == 0) || (input->lidar_fresh == 0U))
    return CHASSIS_SAFETY_LIDAR_STALE;
  if ((input->require_feedback != 0U) &&
      (SongjiaMotor_IsFeedbackFresh(now_ms, input->feedback_timeout_ms) == 0U))
    return CHASSIS_SAFETY_FEEDBACK_STALE;
  if ((input->obstacle_distance_mm > 0.0f) &&
      (input->obstacle_distance_mm <= input->stop_distance_mm))
    return CHASSIS_SAFETY_OBSTACLE;
  return CHASSIS_SAFETY_READY;
}

void AppScheduler_RunChassisEscapeTest(uint32_t now_ms,
                                       const ChassisSafetyInput_t *input)
{
  const ChassisEscapeRequest_t *request;
  ChassisSafetyReason_t reason;
  int16_t motor[4] = {0, 0, 0, 0};

  if ((uint32_t)(now_ms - sEscapeTestTick) < APP_MOTION_PERIOD_MS)
    return;
  sEscapeTestTick = now_ms;
  request = ChassisEscapeTest_GetRequest();
  if ((request->active == 0U) || (input == 0)) return;

  /* Query first: feedback freshness is a safety prerequisite, so it cannot
     depend on a previous successful safety decision. */
  if ((uint32_t)(now_ms - sFeedbackQueryTick) >= APP_FEEDBACK_QUERY_MS)
  {
    sFeedbackQueryTick = now_ms;
    (void)SongjiaMotor_RequestEncoder20ms();
  }
  reason = AppScheduler_CheckChassisEscapeSafety(input, now_ms);
  if (reason != CHASSIS_SAFETY_READY)
  {
    SendRateLimitedSafetyStop(now_ms);
    return;
  }
  motor[0] = request->left_centi_mps;
  motor[1] = request->right_centi_mps;
  (void)SongjiaMotor_SetSpeedCentiMps(motor[0], motor[1], 0, 0);
}
#endif

uint8_t AppScheduler_SendSafeSpeed(const ChassisSafetyInput_t *input,
                                   uint32_t now_ms, const int16_t motor[4])
{
  if ((motor == 0) ||
      (AppScheduler_CheckChassisSafety(input, now_ms) != CHASSIS_SAFETY_READY))
  {
    SendRateLimitedSafetyStop(now_ms);
    return 0U;
  }
  return SongjiaMotor_SetSpeedCentiMps(motor[0], motor[1], motor[2], motor[3]);
}

uint8_t AppScheduler_SendSafePwm(const ChassisSafetyInput_t *input,
                                 uint32_t now_ms, const int16_t motor[4])
{
  if ((motor == 0) ||
      (AppScheduler_CheckChassisSafety(input, now_ms) != CHASSIS_SAFETY_READY))
  {
    SendRateLimitedSafetyStop(now_ms);
    return 0U;
  }
  return SongjiaMotor_SetPwmPermille(motor[0], motor[1], motor[2], motor[3]);
}

void AppScheduler_RunChassisMotion(uint32_t now_ms,
                                   const ChassisSafetyInput_t *input)
{
  const ChassisMotionRequest_t *request = ChassisMotion_GetRequest();
  ChassisSafetyReason_t reason;
  int16_t motor[4] = {0, 0, 0, 0};

  if ((uint32_t)(now_ms - sMotionTick) < APP_MOTION_PERIOD_MS)
    return;
  sMotionTick = now_ms;

  /* This service owns UART7 only while Task4 has an active automatic route.
     Other diagnostic tasks use the same driver and must not receive a
     background stop command or encoder polling traffic from this path. */
  if (request->active == 0U)
    return;

  if (input == 0)
  {
    SendRateLimitedSafetyStop(now_ms);
    return;
  }

  reason = AppScheduler_CheckChassisSafety(input, now_ms);
  ChassisMotion_NotifySafety((reason == CHASSIS_SAFETY_READY) ? 1U : 0U,
                             (uint8_t)reason);
  request = ChassisMotion_GetRequest();

  if ((uint32_t)(now_ms - sFeedbackQueryTick) >= APP_FEEDBACK_QUERY_MS)
  {
    sFeedbackQueryTick = now_ms;
    (void)SongjiaMotor_RequestEncoder20ms();
  }

  motor[0] = request->left_centi_mps;
  motor[1] = request->right_centi_mps;
  (void)AppScheduler_SendSafeSpeed(input, now_ms, motor);
}

void AppScheduler_UpdateStatusLed(uint32_t now_ms)
{
  uint32_t startup_elapsed;

  if ((uint32_t)(now_ms - sLedTick) < 20U)
    return;
  sLedTick = now_ms;
  startup_elapsed = (uint32_t)(now_ms - sStartupDelayTick);

  if (startup_elapsed < APP_STARTUP_DELAY_MS)
  {
    WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, 0U, 0U);  /* red solid: settle 1 s */
    return;
  }
  if ((sApplicationFault != 0U) || (sImu.bias_ready == 0U))
  {
    WS2812_Ctrl(((now_ms / 125U) & 1U) ? WS2812_BRIGHTNESS_MAX : 0U,
                0U, 0U);                                   /* red blink */
    return;
  }
  /* Ready: the task selector or the active task owns the LED. */
}
