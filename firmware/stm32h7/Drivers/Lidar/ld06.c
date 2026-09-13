#include "ld06.h"

#include "main.h"
#include "usart.h"

#include <string.h>

#define LD06_DMA_BUFFER_SIZE 32768U
#define LD06_FRAME_SIZE        47U
#define LD06_PWM_PERIOD      8000U
#define LD06_PWM_PULSE       3200U

static uint8_t dmaBuffer[LD06_DMA_BUFFER_SIZE];
static uint16_t dmaRead;
static uint8_t frame[LD06_FRAME_SIZE];
static uint8_t frameIndex;
static LD06_Data_t workingData;
static LD06_Data_t snapshotData;
static TIM_HandleTypeDef htim2Lidar;
static volatile uint8_t uartRestartNeeded;
static uint16_t previousStartAngle;
static uint8_t havePreviousAngle;
static uint8_t scanSynchronized;
static uint8_t initialized;

static uint16_t ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint8_t Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t i;
  while (length-- != 0U)
  {
    crc ^= *data++;
    for (i = 0U; i < 8U; i++)
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x4DU)
                          : (uint8_t)(crc << 1U);
  }
  return crc;
}

static void PublishScan(uint32_t now_ms)
{
  uint16_t bin;
  uint16_t valid = 0U;
  uint32_t sequence = snapshotData.scan_sequence + 1U;

  for (bin = 0U; bin < LD06_ANGLE_BINS; bin++)
    if ((workingData.distance_mm[bin] != 0U) &&
        (workingData.confidence[bin] != 0U)) valid++;
  memcpy(&snapshotData, &workingData, sizeof(snapshotData));
  snapshotData.scan_sequence = sequence;
  snapshotData.scan_complete_ms = now_ms;
  snapshotData.valid_point_count = valid;
  memset(workingData.distance_mm, 0, sizeof(workingData.distance_mm));
  memset(workingData.confidence, 0, sizeof(workingData.confidence));
}

static void ParseFrame(void)
{
  uint16_t start = ReadU16(&frame[4]);
  uint32_t end = ReadU16(&frame[42]);
  uint32_t span;
  uint8_t point;

  if (Crc8(frame, LD06_FRAME_SIZE - 1U) != frame[LD06_FRAME_SIZE - 1U])
  {
    workingData.crc_error_count++;
    return;
  }
  if ((havePreviousAngle != 0U) && (start + 18000U < previousStartAngle))
  {
    if (scanSynchronized != 0U)
      PublishScan(HAL_GetTick());
    else
    {
      memset(workingData.distance_mm, 0, sizeof(workingData.distance_mm));
      memset(workingData.confidence, 0, sizeof(workingData.confidence));
      scanSynchronized = 1U;
    }
  }
  previousStartAngle = start;
  havePreviousAngle = 1U;
  if (end < start) end += 36000U;
  span = (uint32_t)end - start;
  workingData.speed_dps = ReadU16(&frame[2]);
  for (point = 0U; point < 12U; point++)
  {
    uint32_t angle100 = (uint32_t)start + (span * point) / 11U;
    uint16_t bin = (uint16_t)(((angle100 + 50U) / 100U) % 360U);
    uint8_t offset = (uint8_t)(6U + point * 3U);
    workingData.distance_mm[bin] = ReadU16(&frame[offset]);
    workingData.confidence[bin] = frame[offset + 2U];
  }
  workingData.packet_count++;
  workingData.last_packet_ms = HAL_GetTick();
}

static void FeedByte(uint8_t value)
{
  if (value == 0x54U)
    workingData.sync54_count++;
  if (frameIndex == 0U)
  {
    if (value == 0x54U) frame[frameIndex++] = value;
    return;
  }
  if (frameIndex == 1U)
  {
    if (value == 0x2CU)
    {
      frame[frameIndex++] = value;
      workingData.header_count++;
    }
    else frameIndex = (value == 0x54U) ? 1U : 0U;
    return;
  }
  frame[frameIndex++] = value;
  if (frameIndex == LD06_FRAME_SIZE)
  {
    ParseFrame();
    frameIndex = 0U;
  }
}

static void StartPwm(void)
{
  TIM_OC_InitTypeDef config = {0};
  GPIO_InitTypeDef gpio = {0};

  htim2Lidar.Instance = TIM2;
  htim2Lidar.Init.Prescaler = 0U;
  htim2Lidar.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2Lidar.Init.Period = LD06_PWM_PERIOD - 1U;
  htim2Lidar.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2Lidar.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2Lidar) != HAL_OK) Error_Handler();

  config.OCMode = TIM_OCMODE_PWM1;
  config.Pulse = LD06_PWM_PULSE;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2Lidar, &config, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_TIM_PWM_Start(&htim2Lidar, TIM_CHANNEL_1);
}

uint8_t LD06_Init(void)
{
  HAL_StatusTypeDef status;
  if (initialized != 0U) return 0U;
  memset(&workingData, 0, sizeof(workingData));
  memset(&snapshotData, 0, sizeof(snapshotData));
  memset(dmaBuffer, 0, sizeof(dmaBuffer));
  dmaRead = 0U;
  frameIndex = 0U;
  uartRestartNeeded = 0U;
  havePreviousAngle = 0U;
  scanSynchronized = 0U;
  StartPwm();
  status = HAL_UART_Receive_DMA(&huart5, dmaBuffer, LD06_DMA_BUFFER_SIZE);
  if (status == HAL_OK)
  {
    /* The producer is DMA and the consumer polls NDTR. Half/full callbacks
     * are not needed and would only interrupt application rendering. */
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT | DMA_IT_TC);
    initialized = 1U;
    return 0U;
  }
  return 1U;
}

void LD06_Process(void)
{
  uint16_t write;
  uint16_t budget = LD06_PROCESS_BYTE_BUDGET;
  if (initialized == 0U) return;
  if (uartRestartNeeded != 0U)
  {
    HAL_UART_DMAStop(&huart5);
    dmaRead = 0U;
    frameIndex = 0U;
    havePreviousAngle = 0U;
    scanSynchronized = 0U;
    uartRestartNeeded = 0U;
    if (HAL_UART_Receive_DMA(&huart5, dmaBuffer, LD06_DMA_BUFFER_SIZE) == HAL_OK)
    {
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT | DMA_IT_TC);
      workingData.uart_restart_count++;
    }
  }
  write = (uint16_t)(LD06_DMA_BUFFER_SIZE -
          __HAL_DMA_GET_COUNTER(huart5.hdmarx));
  if (write >= LD06_DMA_BUFFER_SIZE) write = 0U;
  while ((dmaRead != write) && (budget-- != 0U))
  {
    uint8_t value = dmaBuffer[dmaRead];
    workingData.raw_byte_count++;
    if (workingData.raw_sample_count < LD06_RAW_SAMPLE_SIZE)
      workingData.raw_sample[workingData.raw_sample_count++] = value;
    FeedByte(value);
    dmaRead++;
    if (dmaRead >= LD06_DMA_BUFFER_SIZE) dmaRead = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    workingData.last_uart_error = huart->ErrorCode;
    workingData.uart_error_count++;
    uartRestartNeeded = 1U;
  }
}

void LD06_Stop(void)
{
  if (initialized == 0U) return;
  HAL_UART_DMAStop(&huart5);
  __HAL_TIM_SET_COMPARE(&htim2Lidar, TIM_CHANNEL_1, 0U);
  HAL_TIM_PWM_Stop(&htim2Lidar, TIM_CHANNEL_1);
  initialized = 0U;
}

const LD06_Data_t *LD06_GetData(void)
{
  return (snapshotData.scan_sequence != 0U) ? &snapshotData : &workingData;
}

const LD06_Data_t *LD06_GetSnapshot(void)
{
  return &snapshotData;
}

uint8_t LD06_IsFresh(uint32_t now_ms, uint32_t timeout_ms)
{
  return ((snapshotData.scan_sequence != 0U) &&
          ((uint32_t)(now_ms - snapshotData.scan_complete_ms) <= timeout_ms)) ? 1U : 0U;
}
