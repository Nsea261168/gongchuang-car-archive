/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"

/* USER CODE BEGIN 0 */

/* P11 PWM ?? (PA0/PA2/PE9/PE13) ? modules/servo ?????????? */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, POWER_OUT2_EN_Pin | POWER_OUT1_EN_Pin | POWER_5V_EN_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = POWER_OUT2_EN_Pin | POWER_OUT1_EN_Pin | POWER_5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_BLK_Pin | LCD_RES_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LCD_BLK_Pin | LCD_RES_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /* P11 PWM ??? modules/servo ???? task0?? */

  /* Directly copied from the CtrBoard-H7_IMU example: BMI088 chip-select
   * pins idle high and are push-pull, very-high-speed outputs. */
  HAL_GPIO_WritePin(GPIOC, ACC_CS_Pin | GYRO_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = ACC_CS_Pin | GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Board User Key: PA15, active low. */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
