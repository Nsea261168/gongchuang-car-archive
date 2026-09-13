#ifndef CHASSIS_UART3_H
#define CHASSIS_UART3_H

#include <stdint.h>

typedef enum
{
  CHASSIS_LINK_ACCEPTED = 0U,
  CHASSIS_LINK_INVALID_TASK = 1U,
  CHASSIS_LINK_BUSY = 2U
} ChassisLinkStatus_t;

typedef struct
{
  uint8_t task_id;
  uint8_t sequence;
} ChassisLinkRunRequest_t;

void ChassisUart3_Init(void);
void ChassisUart3_IrqHandler(void);
uint8_t ChassisUart3_TakeRunRequest(ChassisLinkRunRequest_t *request);
void ChassisUart3_SendResponse(ChassisLinkStatus_t status, uint8_t task_id,
                                uint8_t sequence);

#endif
