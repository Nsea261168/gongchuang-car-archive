#include "task_ui.h"

#include "lcd.h"

void TaskUi_Show(uint8_t task, const char *title, const char *line1,
                 const char *line2, uint16_t accent)
{
  (void)task;
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_Fill(0U, 0U, 7U, LCD_H, accent);
  LCD_ShowString(22U, 12U, (const uint8_t *)"ZAPPIES", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(22U, 56U, (const uint8_t *)title, WHITE, BLACK, 32U, 0U);
  LCD_DrawLine(22U, 96U, 258U, 96U, accent);
  LCD_ShowString(22U, 120U, (const uint8_t *)line1, accent, BLACK, 24U, 0U);
  LCD_ShowString(22U, 154U, (const uint8_t *)line2, GRAY, BLACK, 16U, 0U);
  LCD_ShowString(22U, 214U, (const uint8_t *)"LEFT  BACK TO MENU",
                 GRAY, BLACK, 16U, 0U);
  LCD_WR_REG(0x29U);
}
