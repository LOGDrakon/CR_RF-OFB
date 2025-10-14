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
#include "stm32wlxx_hal.h"

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
#define SECHEUR_Pin GPIO_PIN_3
#define SECHEUR_GPIO_Port GPIOB
#define CASIER_Pin GPIO_PIN_4
#define CASIER_GPIO_Port GPIOB
#define GAZ_Pin GPIO_PIN_5
#define GAZ_GPIO_Port GPIOB
#define RES_IN_Pin GPIO_PIN_8
#define RES_IN_GPIO_Port GPIOB
#define RES_OUT_Pin GPIO_PIN_0
#define RES_OUT_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOA
#define MASTER_Pin GPIO_PIN_4
#define MASTER_GPIO_Port GPIOA
#define FE_CTRL_Pin GPIO_PIN_6
#define FE_CTRL_GPIO_Port GPIOA
#define ALERT_Pin GPIO_PIN_7
#define ALERT_GPIO_Port GPIOA
#define VENTILATEUR_Pin GPIO_PIN_8
#define VENTILATEUR_GPIO_Port GPIOA
#define BOOT0_Pin GPIO_PIN_3
#define BOOT0_GPIO_Port GPIOH
#define PRESSION_Pin GPIO_PIN_2
#define PRESSION_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
