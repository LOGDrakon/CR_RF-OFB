/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART1_Interrupt(void);
void NEXTION_SendCommand(const char* command);
void NEXTION_SetText(const char* object_name, const char* text);
void NEXTION_SetValue(const char* object_name, int value);
void NEXTION_SetBackgroundColor(const char* object_name, uint16_t color);
void NEXTION_SetTextColor(const char* object_name, uint16_t color);
void NEXTION_ChangePage(uint8_t page_id);
void NEXTION_SetVisible(const char* object_name, uint8_t visible);
void NEXTION_GetValue(const char* object_name);
void NEXTION_ClearScreen(uint16_t color);
void NEXTION_SetPicture(const char* object_name, uint8_t picture);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

