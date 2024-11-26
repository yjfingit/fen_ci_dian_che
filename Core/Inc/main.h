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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define E2A_Pin GPIO_PIN_0
#define E2A_GPIO_Port GPIOA
#define E2B_Pin GPIO_PIN_1
#define E2B_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_9
#define AIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOD
#define E1A_Pin GPIO_PIN_12
#define E1A_GPIO_Port GPIOD
#define E1B_Pin GPIO_PIN_13
#define E1B_GPIO_Port GPIOD
#define st188_1_Pin GPIO_PIN_6
#define st188_1_GPIO_Port GPIOC
#define st188_2_Pin GPIO_PIN_8
#define st188_2_GPIO_Port GPIOC
#define st188_3_Pin GPIO_PIN_8
#define st188_3_GPIO_Port GPIOA
#define st188_4_Pin GPIO_PIN_10
#define st188_4_GPIO_Port GPIOA
#define st188_5_Pin GPIO_PIN_10
#define st188_5_GPIO_Port GPIOC
#define st188_6_Pin GPIO_PIN_12
#define st188_6_GPIO_Port GPIOC
#define st188_7_Pin GPIO_PIN_1
#define st188_7_GPIO_Port GPIOD
#define st188_8_Pin GPIO_PIN_3
#define st188_8_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
