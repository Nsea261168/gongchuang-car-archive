#include "task2.h"

#include "app_scheduler.h"
#include "task_contract.h"
#include "QD4310.h"
#include "servo_pwm.h"
#include "lcd.h"
#include "ws2812.h"

#include <stdio.h>

#define T2_SERVO_UPDATE_MS       20U
#define T2_STAND_MOVE_MS       2000U
#define T2_MOTOR_TEST_MS       3000U
#define T2_MOTOR_COMMAND_MS      20U
#define T2_SCREEN_UPDATE_MS     100U
#define T2_TEST_SPEED_RPM        20.0f  /* one revolution in three seconds */

static const float sVerticalDeg[4] = {90.0f, 90.0f, 90.0f, 90.0f};
static const float sStandDeg[4] = {65.3f, 114.7f, 114.7f, 65.3f};

typedef enum
{
  T2_VERTICAL = 0,
  T2_TEST_A,
  T2_TEST_B,
  T2_TEST_DONE
} Task2State_t;

static Task2State_t sState;
static uint32_t sStateTick;
static uint32_t sServoTick;
static uint32_t sMotorTick;
static uint32_t sScreenTick;

static void Task2_Text(uint16_t x, uint16_t y, const char *text,
                       uint16_t color, uint8_t size)
{
  LCD_ShowString(x, y, (const uint8_t *)text, color, BLACK, size, 0U);
}

static void Task2_Frame(const char *title, const char *hint)
{
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_Fill(0U, 0U, 7U, LCD_H, YELLOW);
  Task2_Text(20U, 10U, "TASK2 / DIRECTION TEST", GRAY, 16U);
  Task2_Text(20U, 38U, title, WHITE, 24U);
  LCD_DrawLine(20U, 68U, 260U, 68U, YELLOW);
  Task2_Text(20U, 216U, hint, GRAY, 16U);
  LCD_WR_REG(0x29U);
}

static void Task2_StopWheels(void)
{
  QD4310_SetSpeed(&Motor_0, 0.0f);
  QD4310_SetSpeed(&Motor_1, 0.0f);
}

static void Task2_CommandState(Task2State_t state)
{
  if (state == T2_TEST_A)
  {
    QD4310_SetSpeed(&Motor_0, T2_TEST_SPEED_RPM);
    QD4310_SetSpeed(&Motor_1, -T2_TEST_SPEED_RPM);
  }
  else if (state == T2_TEST_B)
  {
    QD4310_SetSpeed(&Motor_0, -T2_TEST_SPEED_RPM);
    QD4310_SetSpeed(&Motor_1, T2_TEST_SPEED_RPM);
  }
  else
    Task2_StopWheels();
}

static void Task2_SetVertical(void)
{
  uint8_t i;
  for (i = 0U; i < 4U; i++) ServoPwm_SetAngle((ServoId_t)i, sVerticalDeg[i]);
}

static void Task2_UpdateStand(uint32_t elapsed_ms)
{
  uint8_t i;
  float t = (float)elapsed_ms / (float)T2_STAND_MOVE_MS;
  float blend;
  if (t > 1.0f) t = 1.0f;
  blend = t * t * (3.0f - 2.0f * t);
  for (i = 0U; i < 4U; i++)
    ServoPwm_SetAngle((ServoId_t)i,
        sVerticalDeg[i] + blend * (sStandDeg[i] - sVerticalDeg[i]));
}

static void Task2_DrawVertical(void)
{
  Task2_Text(20U, 84U, "VERTICAL CAL POSE", CYAN, 24U);
  Task2_Text(20U, 122U, "S1 S2 S3 S4 = 90 DEG", WHITE, 16U);
  Task2_Text(20U, 158U, "MOTORS ENABLED / STOPPED", GREEN, 16U);
  Task2_Text(20U, 184U, "CENTER: STAND + START", YELLOW, 16U);
}

static void Task2_DrawTest(Task2State_t state, uint32_t elapsed_ms)
{
  char line[40];
  uint32_t remain_ms = (elapsed_ms >= T2_MOTOR_TEST_MS) ? 0U :
                       T2_MOTOR_TEST_MS - elapsed_ms;
  LCD_Fill(20U, 82U, 260U, 202U, BLACK);
  snprintf(line, sizeof(line), "STATE %c   %lu.%lus LEFT",
           (state == T2_TEST_A) ? 'A' : 'B',
           (unsigned long)(remain_ms / 1000U),
           (unsigned long)((remain_ms % 1000U) / 100U));
  Task2_Text(20U, 84U, line, YELLOW, 24U);
  if (state == T2_TEST_A)
    Task2_Text(20U, 124U, "CMD  LEFT +20  RIGHT -20", CYAN, 16U);
  else
    Task2_Text(20U, 124U, "CMD  LEFT -20  RIGHT +20", CYAN, 16U);
  snprintf(line, sizeof(line), "FB   L %+6.1f  R %+6.1f rpm",
           Motor_0.speed, Motor_1.speed);
  Task2_Text(20U, 150U, line, WHITE, 16U);
  Task2_Text(20U, 180U, "OBSERVE FORWARD / BACKWARD", GREEN, 16U);
}

static void Task2_DrawDone(void)
{
  Task2_Text(20U, 84U, "A: L +20 / R -20", CYAN, 16U);
  Task2_Text(20U, 112U, "B: L -20 / R +20", CYAN, 16U);
  Task2_Text(20U, 146U, "MOTORS STOPPED", GREEN, 24U);
  Task2_Text(20U, 184U, "TELL ME A/B DIRECTION", YELLOW, 16U);
}

static void Task2_Enter(uint32_t now_ms)
{
  sState = T2_VERTICAL;
  sStateTick = now_ms;
  sServoTick = now_ms;
  sMotorTick = now_ms;
  sScreenTick = now_ms;
  ServoPwm_Init();
  Task2_SetVertical();
  /* System boot already enables both motors; repeat here so Task2 is also
   * robust after any earlier task has invoked the global safe stop. */
  QD4310_Enable(&Motor_0);
  QD4310_Enable(&Motor_1);
  Task2_StopWheels();
  WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, WS2812_BRIGHTNESS_MAX, 0U);
  Task2_Frame("VERTICAL CAL POSE", "LEFT: STOP AND EXIT");
  Task2_DrawVertical();
}

static void Task2_Tick(uint32_t now, InputEvent_t event)
{
  uint32_t elapsed = (uint32_t)(now - sStateTick);
  if ((sState == T2_VERTICAL) && (event == INPUT_EVENT_CENTER))
  {
    sState = T2_TEST_A;
    sStateTick = now;
    sServoTick = now;
    sMotorTick = now - T2_MOTOR_COMMAND_MS;
    Task2_Frame("MOTOR TEST A", "LEFT: STOP AND EXIT");
  }
  if ((sState == T2_TEST_A) || (sState == T2_TEST_B))
  {
    elapsed = (uint32_t)(now - sStateTick);
    if ((uint32_t)(now - sServoTick) >= T2_SERVO_UPDATE_MS)
    {
      sServoTick = now;
      Task2_UpdateStand(elapsed);
    }
    if ((uint32_t)(now - sMotorTick) >= T2_MOTOR_COMMAND_MS)
    {
      sMotorTick = now;
      Task2_CommandState(sState);
    }
    if (elapsed >= T2_MOTOR_TEST_MS)
    {
      Task2_StopWheels();
      if (sState == T2_TEST_A)
      {
        sState = T2_TEST_B;
        sStateTick = now;
        sMotorTick = now - T2_MOTOR_COMMAND_MS;
        Task2_Frame("MOTOR TEST B", "LEFT: STOP AND EXIT");
      }
      else
      {
        sState = T2_TEST_DONE;
        Task2_Frame("A/B TEST COMPLETE", "LEFT: EXIT");
        Task2_DrawDone();
      }
    }
  }
  if (((sState == T2_TEST_A) || (sState == T2_TEST_B)) &&
      ((uint32_t)(now - sScreenTick) >= T2_SCREEN_UPDATE_MS))
  {
    sScreenTick = now;
    Task2_DrawTest(sState, (uint32_t)(now - sStateTick));
  }
}

static void Task2_Exit(void)
{
  Task2_StopWheels();
}

const AppTask_t Task2_Definition = {Task2_Enter, Task2_Tick, Task2_Exit};
