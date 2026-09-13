#include "task0.h"

#include "task_contract.h"
#include "ld06.h"
#include "lcd.h"

#include <math.h>

#define PLOT_CX 102
#define PLOT_CY 132
#define PLOT_R   94
#define PLOT_REFRESH_MS 200U

#define RANGE_COUNT 10U
#define RANGE_DEFAULT_INDEX 3U

static const uint16_t rangeMm[RANGE_COUNT] =
{
  12000U, 9000U, 6000U, 3000U, 2000U,
  1000U, 700U, 500U, 300U, 100U
};

static int16_t oldX[LD06_ANGLE_BINS];
static int16_t oldY[LD06_ANGLE_BINS];
static uint8_t rawDebugDrawn;
static uint8_t rangeIndex;
static uint32_t sRefreshTick;

static uint8_t HexDigit(uint8_t value)
{
  value &= 0x0FU;
  return (uint8_t)((value < 10U) ? ('0' + value) : ('A' + value - 10U));
}

static void Task0_UpdateRawStats(const LD06_Data_t *data)
{
  LCD_Fill(80U, 70U, 127U, 107U, BLACK);
  LCD_ShowIntNum(80U, 70U, (uint16_t)(data->sync54_count % 10000U),
                 4U, WHITE, BLACK, 16U);
  LCD_ShowIntNum(80U, 92U, (uint16_t)(data->header_count % 10000U),
                 4U, WHITE, BLACK, 16U);
  LCD_Fill(96U, 114U, 143U, 130U, BLACK);
  LCD_ShowIntNum(96U, 114U, (uint16_t)(data->uart_error_count % 10000U),
                 4U, WHITE, BLACK, 16U);
}

static void Task0_DrawRawDebug(const LD06_Data_t *data)
{
  uint8_t row;
  uint8_t sample;
  uint8_t text[18];

  LCD_Fill(7U, 35U, 197U, 227U, BLACK);
  LCD_ShowString(16U, 45U, (const uint8_t *)"UART5 RAW 230400",
                 CYAN, BLACK, 16U, 0U);
  LCD_ShowString(16U, 70U, (const uint8_t *)"BYTE54", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(16U, 92U, (const uint8_t *)"HEADER", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(16U, 114U, (const uint8_t *)"UART ERR", GRAY, BLACK, 16U, 0U);
  Task0_UpdateRawStats(data);
  for (row = 0U; row < 4U; row++)
  {
    for (sample = 0U; sample < 6U; sample++)
    {
      uint8_t value = data->raw_sample[row * 6U + sample];
      text[sample * 3U] = HexDigit(value >> 4U);
      text[sample * 3U + 1U] = HexDigit(value);
      text[sample * 3U + 2U] = (sample == 5U) ? '\0' : ' ';
    }
    LCD_ShowString(16U, (uint16_t)(142U + row * 20U), text,
                   YELLOW, BLACK, 16U, 0U);
  }
  rawDebugDrawn = 1U;
}

static void Task0_DrawLayout(void)
{
  uint16_t i;
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(10U, 7U, (const uint8_t *)"LD06 LIDAR",
                 CYAN, BLACK, 24U, 0U);
  LCD_DrawRectangle(6U, 34U, 198U, 228U, GRAY);
  LCD_DrawLine(PLOT_CX, 38U, PLOT_CX, 225U, 0x3186U);
  LCD_DrawLine(9U, PLOT_CY, 195U, PLOT_CY, 0x3186U);
  LCD_ShowString(207U, 42U, (const uint8_t *)"LINK", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(207U, 78U, (const uint8_t *)"RPM", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(207U, 114U, (const uint8_t *)"PACK", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(207U, 150U, (const uint8_t *)"CRC", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(207U, 186U, (const uint8_t *)"RANGE", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(207U, 216U, (const uint8_t *)"LEFT", GRAY, BLACK, 16U, 0U);
  for (i = 0U; i < LD06_ANGLE_BINS; i++)
  {
    oldX[i] = -1;
    oldY[i] = -1;
  }
  rawDebugDrawn = 0U;
  LCD_WR_REG(0x29U);
}

static void Task0_UpdateStatus(const LD06_Data_t *data, uint32_t now)
{
  uint8_t online = ((uint32_t)(now - data->last_packet_ms) < 500U) &&
                   (data->packet_count != 0U);
  LCD_Fill(207U, 58U, 279U, 75U, BLACK);
  LCD_ShowString(207U, 58U, (const uint8_t *)(online ? "ONLINE" : "WAIT"),
                 online ? GREEN : YELLOW, BLACK, 16U, 0U);
  LCD_Fill(207U, 94U, 279U, 111U, BLACK);
  LCD_ShowIntNum(207U, 94U, data->speed_dps / 6U, 4U, WHITE, BLACK, 16U);
  LCD_Fill(207U, 130U, 279U, 147U, BLACK);
  LCD_ShowIntNum(207U, 130U, (uint16_t)(data->packet_count % 10000U),
                 4U, WHITE, BLACK, 16U);
  LCD_Fill(207U, 166U, 279U, 183U, BLACK);
  LCD_ShowIntNum(207U, 166U, (uint16_t)(data->crc_error_count % 10000U),
                 4U, WHITE, BLACK, 16U);
  LCD_Fill(207U, 202U, 279U, 215U, BLACK);
  if (rangeMm[rangeIndex] >= 1000U)
  {
    LCD_ShowIntNum(207U, 202U, (uint16_t)(rangeMm[rangeIndex] / 1000U),
                   2U, CYAN, BLACK, 16U);
    LCD_ShowString(231U, 202U, (const uint8_t *)"m", CYAN, BLACK, 16U, 0U);
  }
  else
  {
    LCD_ShowIntNum(207U, 202U, (uint16_t)(rangeMm[rangeIndex] / 10U),
                   2U, CYAN, BLACK, 16U);
    LCD_ShowString(231U, 202U, (const uint8_t *)"cm", CYAN, BLACK, 16U, 0U);
  }
}

static void Task0_DrawPointCloud(const LD06_Data_t *data)
{
  uint16_t angle;
  if (data->packet_count == 0U)
  {
    if ((rawDebugDrawn == 0U) &&
        (data->raw_sample_count == LD06_RAW_SAMPLE_SIZE))
      Task0_DrawRawDebug(data);
    else if (rawDebugDrawn != 0U)
      Task0_UpdateRawStats(data);
    return;
  }
  if (rawDebugDrawn != 0U)
  {
    LCD_Fill(7U, 35U, 197U, 227U, BLACK);
    LCD_DrawLine(PLOT_CX, 38U, PLOT_CX, 225U, 0x3186U);
    LCD_DrawLine(9U, PLOT_CY, 195U, PLOT_CY, 0x3186U);
    rawDebugDrawn = 0U;
  }
  for (angle = 0U; angle < LD06_ANGLE_BINS; angle++)
  {
    if (oldX[angle] >= 0)
      LCD_DrawPoint((uint16_t)oldX[angle], (uint16_t)oldY[angle], BLACK);
    oldX[angle] = -1;
    oldY[angle] = -1;
  }
  for (angle = 0U; angle < LD06_ANGLE_BINS; angle++)
  {
    uint16_t distance = data->distance_mm[angle];
    if ((distance >= 50U) && (distance <= rangeMm[rangeIndex]) &&
        (data->confidence[angle] >= 10U))
    {
      float rad = (float)((angle + 180U) % 360U) * 0.01745329252f;
      float scaled = (float)distance *
                     ((float)PLOT_R / (float)rangeMm[rangeIndex]);
      int16_t x = (int16_t)(PLOT_CX + sinf(rad) * scaled);
      int16_t y = (int16_t)(PLOT_CY - cosf(rad) * scaled);
      if ((x > 7) && (x < 198) && (y > 35) && (y < 228))
      {
        LCD_DrawPoint((uint16_t)x, (uint16_t)y, GREEN);
        oldX[angle] = x;
        oldY[angle] = y;
      }
    }
  }
  LCD_DrawPoint(PLOT_CX, PLOT_CY, RED);
}

static void Task0_Enter(uint32_t now_ms)
{
  sRefreshTick = now_ms;
  rangeIndex = RANGE_DEFAULT_INDEX;
  Task0_DrawLayout();
}

static void Task0_Tick(uint32_t now_ms, InputEvent_t event)
{
  const LD06_Data_t *data;
  if ((event == INPUT_EVENT_UP) && (rangeIndex > 0U)) rangeIndex--;
  if ((event == INPUT_EVENT_DOWN) && (rangeIndex < (RANGE_COUNT - 1U))) rangeIndex++;
  if ((uint32_t)(now_ms - sRefreshTick) < PLOT_REFRESH_MS) return;
  sRefreshTick = now_ms;
  data = LD06_GetSnapshot();
  Task0_UpdateStatus(data, now_ms);
  Task0_DrawPointCloud(data);
}

static void Task0_Exit(void)
{
}

const AppTask_t Task0_Definition = {Task0_Enter, Task0_Tick, Task0_Exit};
