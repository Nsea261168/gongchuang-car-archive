#include "chassis_uart3.h"

#include "main.h"

#define CHASSIS_UART_BAUD 115200U
#define CHASSIS_CMD_HEAD0  0xA5U
#define CHASSIS_CMD_HEAD1  0x5AU
#define CHASSIS_ACK_HEAD0  0x5AU
#define CHASSIS_ACK_HEAD1  0xA5U

typedef enum
{
  RX_WAIT_HEAD0 = 0,
  RX_WAIT_HEAD1,
  RX_TASK,
  RX_SEQUENCE,
  RX_CRC
} ChassisRxState_t;

static volatile ChassisRxState_t rx_state;
static volatile uint8_t rx_task_id;
static volatile uint8_t rx_sequence;
static volatile uint8_t request_pending;
static volatile ChassisLinkRunRequest_t pending_request;

static uint8_t ChassisUart3_Crc8(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0U;
  uint8_t bit;

  while (length-- != 0U)
  {
    crc ^= *data++;
    for (bit = 0U; bit < 8U; bit++)
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
  }
  return crc;
}

void ChassisUart3_Init(void)
{
  GPIO_InitTypeDef gpio_cfg = {0};
  RCC_PeriphCLKInitTypeDef clk_cfg = {0};
  uint32_t pclk_hz;

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  clk_cfg.PeriphClockSelection = RCC_PERIPHCLK_USART234578;
  clk_cfg.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  (void)HAL_RCCEx_PeriphCLKConfig(&clk_cfg);

  gpio_cfg.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio_cfg.Mode = GPIO_MODE_AF_PP;
  gpio_cfg.Pull = GPIO_PULLUP;
  gpio_cfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_cfg.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &gpio_cfg);

  pclk_hz = HAL_RCC_GetPCLK1Freq();
  USART3->CR1 = 0U;
  USART3->CR2 = 0U;
  USART3->CR3 = 0U;
  USART3->BRR = (pclk_hz + (CHASSIS_UART_BAUD / 2U)) / CHASSIS_UART_BAUD;
  USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
  USART3->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

  rx_state = RX_WAIT_HEAD0;
  request_pending = 0U;
  HAL_NVIC_SetPriority(USART3_IRQn, 12U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void ChassisUart3_IrqHandler(void)
{
  uint8_t byte;
  uint8_t frame[4];

  if ((USART3->ISR & USART_ISR_ORE) != 0U)
    USART3->ICR = USART_ICR_ORECF;

  while ((USART3->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    byte = (uint8_t)USART3->RDR;
    switch (rx_state)
    {
      case RX_WAIT_HEAD0:
        if (byte == CHASSIS_CMD_HEAD0) rx_state = RX_WAIT_HEAD1;
        break;
      case RX_WAIT_HEAD1:
        rx_state = (byte == CHASSIS_CMD_HEAD1) ? RX_TASK : RX_WAIT_HEAD0;
        break;
      case RX_TASK:
        rx_task_id = byte;
        rx_state = RX_SEQUENCE;
        break;
      case RX_SEQUENCE:
        rx_sequence = byte;
        rx_state = RX_CRC;
        break;
      case RX_CRC:
        frame[0] = CHASSIS_CMD_HEAD0;
        frame[1] = CHASSIS_CMD_HEAD1;
        frame[2] = rx_task_id;
        frame[3] = rx_sequence;
        if (byte == ChassisUart3_Crc8(frame, sizeof(frame)))
        {
          pending_request.task_id = rx_task_id;
          pending_request.sequence = rx_sequence;
          request_pending = 1U;
        }
        rx_state = RX_WAIT_HEAD0;
        break;
      default:
        rx_state = RX_WAIT_HEAD0;
        break;
    }
  }
}

uint8_t ChassisUart3_TakeRunRequest(ChassisLinkRunRequest_t *request)
{
  if ((request == 0) || (request_pending == 0U))
    return 0U;

  __disable_irq();
  *request = pending_request;
  request_pending = 0U;
  __enable_irq();
  return 1U;
}

void ChassisUart3_SendResponse(ChassisLinkStatus_t status, uint8_t task_id,
                                uint8_t sequence)
{
  uint8_t frame[6];
  uint8_t index;

  frame[0] = CHASSIS_ACK_HEAD0;
  frame[1] = CHASSIS_ACK_HEAD1;
  frame[2] = (uint8_t)status;
  frame[3] = task_id;
  frame[4] = sequence;
  frame[5] = ChassisUart3_Crc8(frame, 5U);

  for (index = 0U; index < sizeof(frame); index++)
  {
    while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U) { }
    USART3->TDR = frame[index];
  }
}
