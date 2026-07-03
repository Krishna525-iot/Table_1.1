/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define RELAY_Pin GPIO_PIN_1
#define RELAY_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_4
#define C1_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_5
#define C2_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_6
#define C3_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_7
#define C4_GPIO_Port GPIOA
#define C5_Pin GPIO_PIN_0
#define C5_GPIO_Port GPIOB
#define C6_Pin GPIO_PIN_1
#define C6_GPIO_Port GPIOB
#define C7_Pin GPIO_PIN_2
#define C7_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_12
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_13
#define R2_GPIO_Port GPIOB
#define R3_Pin GPIO_PIN_14
#define R3_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_15
#define R4_GPIO_Port GPIOB
#define R5_Pin GPIO_PIN_8
#define R5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* Relay control macro
 * RELAY_Pin = PA1, drives ULN2003 IN1 → relay U10 (JY3FF-S-DC5V-C)
 * RELAY_SET(1) = PA1 HIGH = ULN2003 ON = relay energised (contacts closed)
 * RELAY_SET(0) = PA1 LOW  = ULN2003 OFF = relay de-energised (contacts open) */
#define RELAY_SET(x)  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, \
                          (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
