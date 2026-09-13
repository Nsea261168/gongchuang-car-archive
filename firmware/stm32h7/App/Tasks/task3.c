#include "task3.h"

#include "adc_modlue.h"
#include "lcd.h"
#include "main.h"

#include <stdio.h>

#define TASK3_REFRESH_MS 50U
#define TASK3_NO_SAMPLE  0xFFFFU

typedef enum
{
  T3_KEY_NONE = 0,
  T3_KEY_SW2_MID,
  T3_KEY_SW3_ADC_UP,
  T3_KEY_SW4_ADC_DOWN,
  T3_KEY_SW5_ADC_RIGHT,
  T3_KEY_SW6_ADC_LEFT
} Task3KeyClass_t;

static uint32_t sRefreshTick;
static uint16_t sLastPressedAdc;
static Task3KeyClass_t sLastKeyClass;
static InputEvent_t sLastEvent;

static Task3KeyClass_t Task3_GetKeyClass(void)
{
  if (key_mid == 0U)   return T3_KEY_SW2_MID;
  if (key_up == 0U)    return T3_KEY_SW3_ADC_UP;
  if (key_down == 0U)  return T3_KEY_SW4_ADC_DOWN;
  if (key_right == 0U) return T3_KEY_SW5_ADC_RIGHT;
  if (key_left == 0U)  return T3_KEY_SW6_ADC_LEFT;
  return T3_KEY_NONE;
}

static const char *Task3_KeyClassText(Task3KeyClass_t key_class)
{
  switch (key_class)
  {
    case T3_KEY_SW2_MID:       return "SW2 / ADC MID";
    case T3_KEY_SW3_ADC_UP:    return "SW3 / ADC UP";
    case T3_KEY_SW4_ADC_DOWN:  return "SW4 / ADC DOWN";
    case T3_KEY_SW5_ADC_RIGHT: return "SW5 / ADC RIGHT";
    case T3_KEY_SW6_ADC_LEFT:  return "SW6 / ADC LEFT";
    default:                   return "NONE";
  }
}

static const char *Task3_EventText(InputEvent_t event)
{
  switch (event)
  {
    case INPUT_EVENT_UP:     return "UP";
    case INPUT_EVENT_DOWN:   return "DOWN";
    case INPUT_EVENT_LEFT:   return "LEFT";
    case INPUT_EVENT_RIGHT:  return "RIGHT";
    case INPUT_EVENT_CENTER: return "CENTER";
    default:                 return "NONE";
  }
}

static void Task3_ShowValue(uint16_t y, const char *label, const char *value,
                            uint16_t color)
{
  LCD_Fill(16U, y, 270U, (uint16_t)(y + 19U), BLACK);
  LCD_ShowString(18U, y, (const uint8_t *)label, GRAY, BLACK, 16U, 0U);
  LCD_ShowString(122U, y, (const uint8_t *)value, color, BLACK, 16U, 0U);
}

static void Task3_DrawFrame(void)
{
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_Fill(0U, 0U, 7U, LCD_H, CYAN);
  LCD_ShowString(18U, 10U, (const uint8_t *)"TASK3 / ADC KEY TEST",
                 WHITE, BLACK, 24U, 0U);
  LCD_DrawLine(18U, 38U, 268U, 38U, CYAN);
  LCD_ShowString(18U, 148U, (const uint8_t *)"SW2 MID   SW3 ADC-UP",
                 GRAY, BLACK, 16U, 0U);
  LCD_ShowString(18U, 168U, (const uint8_t *)"SW4 ADC-DOWN",
                 GRAY, BLACK, 16U, 0U);
  LCD_ShowString(18U, 188U, (const uint8_t *)"SW5 ADC-RIGHT  SW6 ADC-LEFT",
                 GRAY, BLACK, 16U, 0U);
  LCD_ShowString(18U, 220U, (const uint8_t *)"HOLD LEFT; RELEASE EXITS",
                 YELLOW, BLACK, 16U, 0U);
  LCD_WR_REG(0x29U);
}

static void Task3_Enter(uint32_t now_ms)
{
  sRefreshTick = now_ms - TASK3_REFRESH_MS;
  sLastPressedAdc = TASK3_NO_SAMPLE;
  sLastKeyClass = T3_KEY_NONE;
  sLastEvent = INPUT_EVENT_NONE;
  Task3_DrawFrame();
}

static void Task3_Tick(uint32_t now_ms, InputEvent_t event)
{
  char value[24];
  uint16_t live_adc = adc_val[1];
  Task3KeyClass_t live_class = Task3_GetKeyClass();

  if (live_class != T3_KEY_NONE)
  {
    sLastPressedAdc = live_adc;
    sLastKeyClass = live_class;
  }
  if (event != INPUT_EVENT_NONE)
    sLastEvent = event;

  if ((uint32_t)(now_ms - sRefreshTick) < TASK3_REFRESH_MS)
    return;
  sRefreshTick = now_ms;

  snprintf(value, sizeof(value), "%u", (unsigned int)live_adc);
  Task3_ShowValue(48U, "LIVE ADC", value, CYAN);
  if (sLastPressedAdc == TASK3_NO_SAMPLE)
    snprintf(value, sizeof(value), "----");
  else
    snprintf(value, sizeof(value), "%u", (unsigned int)sLastPressedAdc);
  Task3_ShowValue(72U, "LAST PRESS", value, WHITE);
  Task3_ShowValue(96U, "ADC CLASS", Task3_KeyClassText(sLastKeyClass), GREEN);
  Task3_ShowValue(120U, "APP EVENT", Task3_EventText(sLastEvent), YELLOW);
}

static void Task3_Exit(void) {}
const AppTask_t Task3_Definition = {Task3_Enter, Task3_Tick, Task3_Exit};
