/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint16_t adc_val[2];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_OUT2_EN_Pin GPIO_PIN_13
#define POWER_OUT2_EN_GPIO_Port GPIOC
#define POWER_OUT1_EN_Pin GPIO_PIN_14
#define POWER_OUT1_EN_GPIO_Port GPIOC
#define POWER_5V_EN_Pin GPIO_PIN_15
#define POWER_5V_EN_GPIO_Port GPIOC
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA

/* Pin names expected by the unmodified DM CtrBoard-H7_IMU BMI088 example. */
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC

/* DM-MC02 LCD: pins used by the official CtrBoard-H7_ALL LCD example. */
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOE
#define LCD_BLK_Pin GPIO_PIN_10
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_11
#define LCD_RES_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
