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
#define WS2812_Pin GPIO_PIN_2
#define WS2812_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOE
#define MT6701_CSN_Pin GPIO_PIN_4
#define MT6701_CSN_GPIO_Port GPIOA
#define MT6701_CSN2_Pin GPIO_PIN_12
#define MT6701_CSN2_GPIO_Port GPIOB
#define KEY_Up_Pin GPIO_PIN_8
#define KEY_Up_GPIO_Port GPIOD
#define KEY_Right_Pin GPIO_PIN_9
#define KEY_Right_GPIO_Port GPIOD
#define KEY_Down_Pin GPIO_PIN_10
#define KEY_Down_GPIO_Port GPIOD
#define KEY_Left_Pin GPIO_PIN_11
#define KEY_Left_GPIO_Port GPIOD
#define KEY_OK_Pin GPIO_PIN_12
#define KEY_OK_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOD
#define W25_NSS_Pin GPIO_PIN_15
#define W25_NSS_GPIO_Port GPIOD
#define TFT3_CS_Pin GPIO_PIN_15
#define TFT3_CS_GPIO_Port GPIOA
#define TFT3_DC_Pin GPIO_PIN_2
#define TFT3_DC_GPIO_Port GPIOD
#define TFT3_RESET_Pin GPIO_PIN_3
#define TFT3_RESET_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
