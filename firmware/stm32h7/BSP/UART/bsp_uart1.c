#include "bsp_uart7.h"

#include "main.h"

#define UART7_BAUD_RATE       115200U
#define UART7_RX_BUFFER_SIZE  128U
#define UART7_TX_BUFFER_SIZE  256U

static volatile uint8_t sRxBuffer[UART7_RX_BUFFER_SIZE];
static volatile uint16_t sRxHead;
static volatile uint16_t sRxTail;
static volatile uint8_t sTxBuffer[UART7_TX_BUFFER_SIZE];
static volatile uint16_t sTxHead;
static volatile uint16_t sTxTail;
static volatile uint32_t sRxOverflowCount;
static volatile uint32_t sTxDropCount;

static uint16_t NextIndex(uint16_t index, uint16_t size)
{
  index++;
  return (index >= size) ? 0U : index;
}

void BspUart7_Init(void)
{
  GPIO_InitTypeDef gpio_cfg = {0};
  RCC_PeriphCLKInitTypeDef clock_cfg = {0};
  uint32_t pclk_hz;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_UART7_CLK_ENABLE();

  clock_cfg.PeriphClockSelection = RCC_PERIPHCLK_UART7;
  clock_cfg.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&clock_cfg) != HAL_OK)
    Error_Handler();

  gpio_cfg.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  gpio_cfg.Mode = GPIO_MODE_AF_PP;
  gpio_cfg.Pull = GPIO_PULLUP;
  gpio_cfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_cfg.Alternate = GPIO_AF7_UART7;
  HAL_GPIO_Init(GPIOE, &gpio_cfg);

  pclk_hz = HAL_RCC_GetPCLK1Freq();
  UART7->CR1 = 0U;
  UART7->CR2 = 0U;
  UART7->CR3 = 0U;
  UART7->BRR = (pclk_hz + (UART7_BAUD_RATE / 2U)) / UART7_BAUD_RATE;
  UART7->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;

  sRxHead = 0U;
  sRxTail = 0U;
  sTxHead = 0U;
  sTxTail = 0U;
  sRxOverflowCount = 0U;
  sTxDropCount = 0U;

  UART7->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE |
                USART_CR1_UE;
  HAL_NVIC_SetPriority(UART7_IRQn, 12U, 0U);
  HAL_NVIC_EnableIRQ(UART7_IRQn);
}

void BspUart7_IrqHandler(void)
{
  if ((UART7->ISR & USART_ISR_ORE) != 0U)
    UART7->ICR = USART_ICR_ORECF;

  while ((UART7->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    uint8_t byte = (uint8_t)UART7->RDR;
    uint16_t next = NextIndex(sRxHead, UART7_RX_BUFFER_SIZE);
    if (next == sRxTail)
      sRxOverflowCount++;
    else
    {
      sRxBuffer[sRxHead] = byte;
      sRxHead = next;
    }
  }

  if (((UART7->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U) &&
      ((UART7->ISR & USART_ISR_TXE_TXFNF) != 0U))
  {
    if (sTxTail != sTxHead)
    {
      UART7->TDR = sTxBuffer[sTxTail];
      sTxTail = NextIndex(sTxTail, UART7_TX_BUFFER_SIZE);
    }
    else
    {
      CLEAR_BIT(UART7->CR1, USART_CR1_TXEIE_TXFNFIE);
    }
  }
}

uint8_t BspUart7_ReadByte(uint8_t *byte)
{
  if ((byte == 0) || (sRxTail == sRxHead))
    return 0U;

  *byte = sRxBuffer[sRxTail];
  sRxTail = NextIndex(sRxTail, UART7_RX_BUFFER_SIZE);
  return 1U;
}

uint8_t BspUart7_Write(const uint8_t *data, uint16_t length)
{
  uint16_t index;
  uint16_t used;
  uint16_t free_space;

  if ((data == 0) || (length == 0U) || (length >= UART7_TX_BUFFER_SIZE))
    return 0U;

  __disable_irq();
  used = (sTxHead >= sTxTail) ? (uint16_t)(sTxHead - sTxTail) :
                               (uint16_t)(UART7_TX_BUFFER_SIZE - sTxTail + sTxHead);
  free_space = (uint16_t)(UART7_TX_BUFFER_SIZE - 1U - used);
  if (length > free_space)
  {
    sTxDropCount++;
    __enable_irq();
    return 0U;
  }

  for (index = 0U; index < length; index++)
  {
    sTxBuffer[sTxHead] = data[index];
    sTxHead = NextIndex(sTxHead, UART7_TX_BUFFER_SIZE);
  }
  SET_BIT(UART7->CR1, USART_CR1_TXEIE_TXFNFIE);
  __enable_irq();
  return 1U;
}

uint32_t BspUart7_GetRxOverflowCount(void)
{
  return sRxOverflowCount;
}

uint32_t BspUart7_GetTxDropCount(void)
{
  return sTxDropCount;
}
