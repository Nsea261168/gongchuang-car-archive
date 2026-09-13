#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdint.h>

void TaskManager_Init(void);
void TaskManager_Tick(uint32_t now_ms);

#endif
