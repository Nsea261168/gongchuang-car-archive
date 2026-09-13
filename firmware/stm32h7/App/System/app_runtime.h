#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>

void AppRuntime_Init(void);
void AppRuntime_Tick(uint32_t now_ms);
uint8_t AppRuntime_IsReady(void);

#endif
