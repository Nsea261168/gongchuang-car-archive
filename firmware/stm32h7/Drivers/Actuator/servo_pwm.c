/* Drivers/Actuator: TIM1/TIM2 servo PWM driver.
 *
 * 初始化流程照抄工程 CtrBoard-H7_TIM 例程（Core/Src/tim.c + main.c）：
 *   MX_TIMx_Init 风格 HAL 配置 + HAL_TIM_PWM_Start；
 *   引脚 PE9/PE13（TIM1_CH1/CH3）、PA0/PA2（TIM2_CH1/CH3）与例程一致。
 * 仅把例程的 10 kHz 演示参数改为舵机 50 Hz：1 MHz 计数 / 20 ms 周期，
 * 脉宽 500..2500 us 对应 0..180 deg。
 */
#include "servo_pwm.h"

#include "main.h"

#define SERVO_TIMER_HZ        1000000UL  /* 1 MHz counting clock */
#define SERVO_FRAME_US        20000UL    /* 20 ms / 50 Hz */
#define SERVO_MIN_PULSE_US    500U
#define SERVO_MAX_PULSE_US    2500U
#define SERVO_CENTER_PULSE_US 1500U

typedef struct
{
  TIM_TypeDef *tim;
  uint32_t channel;
} ServoChannel_t;

static TIM_HandleTypeDef sHtim1;
static TIM_HandleTypeDef sHtim2;

static const ServoChannel_t sChannels[SERVO_COUNT] = {
  { TIM2, TIM_CHANNEL_1 },  /* SERVO_1: PA0  */
  { TIM2, TIM_CHANNEL_3 },  /* SERVO_2: PA2  */
  { TIM1, TIM_CHANNEL_1 },  /* SERVO_3: PE9  */
  { TIM1, TIM_CHANNEL_3 },  /* SERVO_4: PE13 */
};

/* ---- 照抄 CtrBoard-H7_TIM 例程 tim.c：HAL MSP 回调 ---- */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (tim_pwmHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

/* 例程中这段在 HAL_TIM_MspPostInit 里（CubeMX 生成，非 HAL 弱函数）。
 * 这里做成模块内静态函数，避免以后生成 tim.c 时符号冲突。 */
static void Servo_TimMspPostInit(TIM_HandleTypeDef *timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (timHandle->Instance == TIM1)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /* PE9 ------> TIM1_CH1, PE13 ------> TIM1_CH3 */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA0 ------> TIM2_CH1, PA2 ------> TIM2_CH3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/* ---- 照抄例程 MX_TIM1_Init，仅改 PSC/ARR ---- */
static void Servo_InitTimer1(uint32_t psc, uint32_t arr)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  sHtim1.Instance = TIM1;
  sHtim1.Init.Prescaler = psc;
  sHtim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  sHtim1.Init.Period = arr;
  sHtim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sHtim1.Init.RepetitionCounter = 0;
  sHtim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&sHtim1) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&sHtim1, &sMasterConfig) != HAL_OK)
    Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_PULSE_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&sHtim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&sHtim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&sHtim1, &sBreakDeadTimeConfig) != HAL_OK)
    Error_Handler();

  Servo_TimMspPostInit(&sHtim1);
}

/* ---- 照抄例程 MX_TIM2_Init，仅改 PSC/ARR ---- */
static void Servo_InitTimer2(uint32_t psc, uint32_t arr)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  sHtim2.Instance = TIM2;
  sHtim2.Init.Prescaler = psc;
  sHtim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  sHtim2.Init.Period = arr;
  sHtim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sHtim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&sHtim2) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&sHtim2, &sMasterConfig) != HAL_OK)
    Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_PULSE_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&sHtim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&sHtim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    Error_Handler();

  Servo_TimMspPostInit(&sHtim2);
}

static uint32_t Servo_TimerClockHz(TIM_TypeDef *tim)
{
  /* APB 分频 >1 时定时器时钟 = 2 * APB 时钟；例程按 240 MHz 验证。 */
  uint32_t pclk = (tim == TIM1) ? HAL_RCC_GetPCLK2Freq()
                                 : HAL_RCC_GetPCLK1Freq();
  return pclk * 2U;
}

uint8_t ServoPwm_Init(void)
{
  uint32_t psc = Servo_TimerClockHz(TIM1) / SERVO_TIMER_HZ - 1U;  /* 239 */
  uint32_t arr = SERVO_FRAME_US - 1U;                             /* 19999 */

  Servo_InitTimer1(psc, arr);
  Servo_InitTimer2(psc, arr);

  /* 照抄例程 main.c：启动四路 PWM 输出。 */
  HAL_TIM_PWM_Start(&sHtim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&sHtim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&sHtim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&sHtim2, TIM_CHANNEL_3);
  return 0U;
}

static uint32_t Servo_ClampU32(uint32_t value, uint32_t lo, uint32_t hi)
{
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

void ServoPwm_SetPulseUs(ServoId_t servo, uint16_t pulse_us)
{
  const ServoChannel_t *ch;
  uint32_t ccr;

  if (servo >= SERVO_COUNT)
    return;
  ccr = Servo_ClampU32(pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  ch = &sChannels[servo];
  /* 与例程 main.c 一致的直接 CCR 写法。 */
  if (ch->channel == TIM_CHANNEL_1)
    ch->tim->CCR1 = ccr;
  else if (ch->channel == TIM_CHANNEL_3)
    ch->tim->CCR3 = ccr;
}

void ServoPwm_SetAngle(ServoId_t servo, float angle_deg)
{
  uint16_t pulse_us;

  if (angle_deg < 0.0f) angle_deg = 0.0f;
  if (angle_deg > 180.0f) angle_deg = 180.0f;
  pulse_us = (uint16_t)(SERVO_MIN_PULSE_US +
                        (angle_deg / 180.0f) *
                        (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
  ServoPwm_SetPulseUs(servo, pulse_us);
}
