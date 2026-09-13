#ifndef INPUT_SERVICE_H
#define INPUT_SERVICE_H

#include <stdint.h>

typedef enum
{
  INPUT_EVENT_NONE = 0,
  INPUT_EVENT_UP,
  INPUT_EVENT_DOWN,
  INPUT_EVENT_LEFT,
  INPUT_EVENT_RIGHT,
  INPUT_EVENT_CENTER
} InputEvent_t;

void InputService_Init(void);
void InputService_Poll(void);
InputEvent_t InputService_TakeEvent(void);

#endif
