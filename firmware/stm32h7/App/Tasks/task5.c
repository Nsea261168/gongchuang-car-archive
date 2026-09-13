#include "task5.h"

#include "task_ui.h"
#include "lcd.h"

static void Task5_Enter(uint32_t now_ms)
{
  (void)now_ms;
  TaskUi_Show(5U, "RESERVED 5", "EMPTY SLOT", "MODULE NOT ASSIGNED", GBLUE);
}
static void Task5_Tick(uint32_t now_ms, InputEvent_t event)
{ (void)now_ms; (void)event; }
static void Task5_Exit(void) {}
const AppTask_t Task5_Definition = {Task5_Enter, Task5_Tick, Task5_Exit};
