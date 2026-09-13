#include "boot_ui.h"

#include "main.h"
#include "lcd.h"
#include "zappies_logo.h"

static uint8_t logoLine[ZAPPIES_LOGO_W * 2U];

void BootUi_ShowSplash(void)
{
  uint16_t x, y;
  uint16_t x0 = (LCD_W - ZAPPIES_LOGO_W) / 2U;
  uint16_t y0 = (LCD_H - ZAPPIES_LOGO_H) / 2U;

  LCD_Fill(0U, 0U, LCD_W, LCD_H, WHITE);
  for (y = 0U; y < ZAPPIES_LOGO_H; y++)
  {
    for (x = 0U; x < ZAPPIES_LOGO_W; x++)
    {
      uint32_t pixel = (uint32_t)y * ZAPPIES_LOGO_W + x;
      uint8_t black = (zappiesLogoBits[pixel >> 3U] >> (pixel & 7U)) & 1U;
      uint16_t color = black ? BLACK : WHITE;
      logoLine[x * 2U] = (uint8_t)(color >> 8U);
      logoLine[x * 2U + 1U] = (uint8_t)color;
    }
    LCD_ShowPicture(x0, (uint16_t)(y0 + y), ZAPPIES_LOGO_W, 1U, logoLine);
  }
}

void BootUi_ShowGyroCalibration(void)
{
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(26U, 104U, (const uint8_t *)"Gyro Calibrating...",
                 WHITE, BLACK, 24U, 0U);
}
