#include "task_manager.h"

#include "app_runtime.h"
#include "app_scheduler.h"
#include "input_service.h"
#include "lcd.h"
#include "task_contract.h"
#include "task0.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "task5.h"
#include "task6.h"

#define TASK_COUNT 7U

static const char * const sTaskNames[TASK_COUNT] =
{
  "MG513 SONGJIA", "IMU AHRS", "BALANCE CAL", "ADC KEY TEST",
  "LIDAR MAP", "RESERVED 5", "LD06 LIDAR"
};

static const AppTask_t * const sTasks[TASK_COUNT] =
{
  &Mg513_Definition, &Task1_Definition, &Task2_Definition, &Task3_Definition,
  &Task4_Definition, &Task5_Definition, &Task0_Definition
};

static uint8_t sSelectedTask;
static const AppTask_t *sActiveTask;
static uint8_t sMenuDrawn;

static void DrawTaskRow(uint8_t task, uint8_t selected)
{
  uint16_t y = (uint16_t)(39U + task * 25U);
  uint16_t fg = selected ? WHITE : GRAY;
  LCD_Fill(16U, (uint16_t)(y - 1U), 264U, (uint16_t)(y + 25U), BLACK);
  if (selected != 0U)
  {
    LCD_Fill(18U, y, 262U, (uint16_t)(y + 2U), CYAN);
    LCD_Fill(18U, (uint16_t)(y + 21U), 262U, (uint16_t)(y + 23U), CYAN);
    LCD_Fill(18U, y, 20U, (uint16_t)(y + 23U), CYAN);
    LCD_Fill(260U, y, 262U, (uint16_t)(y + 23U), CYAN);
  }
  LCD_ShowString(32U, (uint16_t)(y + 3U),
                 (const uint8_t *)sTaskNames[task], fg, BLACK, 16U, 0U);
}

static void DrawMenu(void)
{
  uint8_t task;
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(18U, 10U, (const uint8_t *)"ZAPPIES / SELECT", CYAN,
                 BLACK, 24U, 0U);
  LCD_ShowString(32U, 225U, (const uint8_t *)"UP/DOWN     RIGHT GO",
                 GRAY, BLACK, 16U, 0U);
  for (task = 0U; task < TASK_COUNT; task++)
    DrawTaskRow(task, (task == sSelectedTask) ? 1U : 0U);
  LCD_WR_REG(0x29U);
}

void TaskManager_Init(void)
{
  sSelectedTask = 0U;
  sActiveTask = 0;
  sMenuDrawn = 0U;
}

void TaskManager_Tick(uint32_t now_ms)
{
  InputEvent_t event;

  if (AppRuntime_IsReady() == 0U)
    return;

  event = InputService_TakeEvent();
  if (sActiveTask != 0)
  {
    if (event == INPUT_EVENT_LEFT)
    {
      sActiveTask->Exit();
      AppScheduler_SafeStop();
      sActiveTask = 0;
      sMenuDrawn = 0U;
      return;
    }
    sActiveTask->Tick(now_ms, event);
    return;
  }

  if (sMenuDrawn == 0U)
  {
    DrawMenu();
    sMenuDrawn = 1U;
  }

  if (event == INPUT_EVENT_UP)
  {
    uint8_t old = sSelectedTask;
    sSelectedTask = (sSelectedTask == 0U) ? (TASK_COUNT - 1U) :
                                             (uint8_t)(sSelectedTask - 1U);
    DrawTaskRow(old, 0U);
    DrawTaskRow(sSelectedTask, 1U);
  }
  else if (event == INPUT_EVENT_DOWN)
  {
    uint8_t old = sSelectedTask;
    sSelectedTask = (uint8_t)((sSelectedTask + 1U) % TASK_COUNT);
    DrawTaskRow(old, 0U);
    DrawTaskRow(sSelectedTask, 1U);
  }
  else if (event == INPUT_EVENT_RIGHT)
  {
    sActiveTask = sTasks[sSelectedTask];
    sActiveTask->Enter(now_ms);
  }
}
