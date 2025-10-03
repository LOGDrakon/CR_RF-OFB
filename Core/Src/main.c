/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "iwdg.h"
#include "rtc.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>
#include <time.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "subghz_phy_app.h"
#include "tmp275_driver.h"

// Add extern declarations for TMP275 sensors
extern TMP275_Handle_t tmp_sensor_1;
extern TMP275_Handle_t tmp_sensor_2;
extern TMP275_Handle_t tmp_sensor_3;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM2_Init(void);
void TIM2_IRQHandler(void);
void TIM16_Init(void);
void TIM16_IRQHandler(void);
void APP_TMP275_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	// Initialisation différée du générateur aléatoire (après init GPIO)

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	{
		// Initialiser le générateur aléatoire avec plusieurs sources d'entropie
		uint32_t entropy = HAL_GetTick();
		entropy ^= HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
		// Lire l'état des GPIO uniquement après activation des horloges GPIO
		entropy ^= ((uint32_t)GPIOA->IDR << 16) ^ (uint32_t)GPIOB->IDR;
		srand(entropy);
	}
	MX_ADC_Init();
	MX_SubGHz_Phy_Init();
	MX_USART1_UART_Init();
	MX_IWDG_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	APP_TMP275_Init();

	TIM2_Init();
	if(!HAL_GPIO_ReadPin(MASTER_GPIO_Port, MASTER_Pin)) TIM16_Init();

	__enable_irq();

	SubghzApp_Start();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		/* USER CODE END WHILE */
		MX_SubGHz_Phy_Process();

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
			|RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
			|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
			|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void TIM2_Init(void)
{
	// Configure TIM2 for 5-second period using direct registers
	__HAL_RCC_TIM2_CLK_ENABLE();

	// 32MHz / 32000 = 1kHz tick
	TIM2->PSC = 31999;
	TIM2->ARR = 5000; // 5 seconds at 1kHz
	TIM2->CNT = 0;
	TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable

	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Start TIM2 only for transmitters (MASTER pin high)
	if (HAL_GPIO_ReadPin(MASTER_GPIO_Port, MASTER_Pin)) {
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void TIM16_Init(void)
{
	// Configure TIM16 for 10-second timeout using direct registers
	__HAL_RCC_TIM16_CLK_ENABLE();

	TIM16->PSC = 31999; // 32MHz / 32000 = 1kHz
	TIM16->ARR = 10000; // 10 seconds at 1kHz
	TIM16->CNT = 0;
	TIM16->DIER |= TIM_DIER_UIE; // Update interrupt enable

	HAL_NVIC_SetPriority(TIM16_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM16_IRQn);

	TIM16->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF; // clear update flag
		SubghzApp_Event();
	}
}

void TIM16_IRQHandler(void)
{
	if (TIM16->SR & TIM_SR_UIF) {
		TIM16->SR &= ~TIM_SR_UIF; // clear update flag
		SubGHz_Phy_Timeout();
	}
}

// HAL Timer callbacks not used (timers configured via registers)

void APP_TMP275_Init(void)
{
	TMP275_GPIO_Init();
	TMP275_I2C_Init();

	TMP275_Init(&tmp_sensor_1);
	TMP275_Init(&tmp_sensor_2);
	TMP275_Init(&tmp_sensor_3);
}

int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */