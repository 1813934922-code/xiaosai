/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_it.h"
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
#define AD1_Pin GPIO_PIN_6
#define AD1_GPIO_Port GPIOE
#define led3_Pin GPIO_PIN_13
#define led3_GPIO_Port GPIOC
#define AD0_Pin GPIO_PIN_9
#define AD0_GPIO_Port GPIOF
#define led1_Pin GPIO_PIN_0
#define led1_GPIO_Port GPIOC
#define buzzer_Pin GPIO_PIN_1
#define buzzer_GPIO_Port GPIOC
#define led2_Pin GPIO_PIN_2
#define led2_GPIO_Port GPIOC
#define AD2_Pin GPIO_PIN_3
#define AD2_GPIO_Port GPIOC
#define key1_Pin GPIO_PIN_11
#define key1_GPIO_Port GPIOB
#define M1_D2_Pin GPIO_PIN_10
#define M1_D2_GPIO_Port GPIOA
#define M2_D1_Pin GPIO_PIN_11
#define M2_D1_GPIO_Port GPIOA
#define M2_D2_Pin GPIO_PIN_12
#define M2_D2_GPIO_Port GPIOA
#define M1_D1_Pin GPIO_PIN_12
#define M1_D1_GPIO_Port GPIOC
#define key2_Pin GPIO_PIN_2
#define key2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
