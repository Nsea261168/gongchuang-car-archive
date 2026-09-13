#ifndef TASK_CONTRACT_H
#define TASK_CONTRACT_H

#include <stdint.h>
#include "input_service.h"

typedef struct
{
  void (*Enter)(uint32_t now_ms);
  void (*Tick)(uint32_t now_ms, InputEvent_t event);
  void (*Exit)(void);
} AppTask_t;

#endif
