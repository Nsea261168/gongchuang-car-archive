#ifndef TASK_UI_H
#define TASK_UI_H

#include <stdint.h>

void TaskUi_Show(uint8_t task, const char *title, const char *line1,
                 const char *line2, uint16_t accent);

#endif
