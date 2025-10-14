/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    subghz_phy_app.h
 * @author  MCD Application Team
 * @brief   Header of application of the SubGHz_Phy Middleware
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
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	SUBGHZ_APP_IDLE 			= 0x00,
	SUBGHZ_APP_MASTER_1_RX 		= 0x01,
	SUBGHZ_APP_MASTER_1_TX 		= 0x02,
	SUBGHZ_APP_MASTER_2_RX 		= 0x03,
	SUBGHZ_APP_MASTER_2_TX 		= 0x04,
	SUBGHZ_APP_SLAVE_RX 		= 0x05,
	SUBGHZ_APP_SLAVE_TX 		= 0x06
} SubghzApp_State_t;

typedef enum {
	SUBGHZ_APP_SLAVE			= 0x00,
	SUBGHZ_APP_MASTER_1			= 0x01,
	SUBGHZ_APP_MASTER_2			= 0x02
} SubghzApp_Mode_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Subghz Application
  */
void SubghzApp_Init(void);

/* USER CODE BEGIN EFP */
void SubghzApp_Timeout(void);
void SubghzApp_Event(void);
void SubghzApp_UART_RxDone(uint8_t *rxBuffer, uint16_t rxLen);
void SubghzApp_RxProcess(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__SUBGHZ_PHY_APP_H__*/
