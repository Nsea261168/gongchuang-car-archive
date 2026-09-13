/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hardware startup and application scheduler.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "app_runtime.h"
#include "task_manager.h"
#include "chassis_uart3.h"
#include "lcd.h"
#include "adc_modlue.h"
#include "input_service.h"
/* USER CODE END Includes */

uint16_t adc_val[2];

void SystemClock_Config(void);

int main(void)
{
  SCB_EnableICache();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();

  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2);
  ChassisUart3_Init();
  AppRuntime_Init();
  InputService_Init();
  TaskManager_Init();

  while (1)
  {
    uint32_t now = HAL_GetTick();
    AppRuntime_Tick(now);
    InputService_Poll();
    TaskManager_Tick(now);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  /* Match the official CtrBoard-H7_ALL clock tree: SPI1 uses PLL-Q. */
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
