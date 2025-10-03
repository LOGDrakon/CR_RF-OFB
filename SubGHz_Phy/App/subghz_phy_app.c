/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    subghz_phy_app.c
 * @author  MCD Application Team
 * @brief   Application of the SubGHz_Phy Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "tmp275_driver.h"
#include "nextion_driver.h"
// #include "radio_driver.h"  // Fichier manquant - à créer si nécessaire
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
RadioState_t RadioState;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RX_TIMEOUT				3000
#define APP_TX_TIMEOUT				500

#define APP_FREQUENCY				868000000
#define APP_PAYLOAD_LENGTH			24

#define APP_MODEM					0x1
#define APP_OUTPUT_POWER			22
#define APP_BANDWIDTH				0				//[0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
#define	APP_SPREADING_FACTOR		10
#define APP_CODING_RATE				4				//[1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define	APP_PREAMBLE_LENGTH			12
#define	APP_HEADER					0
#define APP_CRC						1
#define APP_SYMB_TIMEOUT			6000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
uint8_t SUBGHZ_APP_ID[9] = "SAGV-CRRF";
SubghzApp_Mode_t SubghzApp_Mode = SUBGHZ_APP_MASTER_1;
uint8_t SubghzApp_State_n = 0;

uint8_t RxBuffer[APP_PAYLOAD_LENGTH];
uint8_t buffer[APP_PAYLOAD_LENGTH];

uint8_t seuil;
uint8_t currentPage;

SubghzApp_State_t SubghzApp_State;

uint8_t past_errors = 0;
uint8_t *redeem_USART_ptr = NULL;
int redeem_USART_length = 0;
int redeem_done = 1;

/* Définition de l'identité de l'émetteur (0, 1 ou 2) */
uint8_t sender_id = 0; // À définir selon l'émetteur (0, 1 ou 2)

// Non-blocking TX jitter scheduling
static volatile uint8_t tx_jitter_pending = 0;
static volatile uint32_t tx_due_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
 * @brief Function to be executed on Radio Rx Done event
 * @param  payload ptr of buffer received
 * @param  size buffer size
 * @param  rssi
 * @param  LoraSnr_FskCfo
 */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
 * @brief Function executed on Radio Tx Timeout event
 */
static void OnTxTimeout(void);

/**
 * @brief Function executed on Radio Rx Timeout event
 */
static void OnRxTimeout(void);

/**
 * @brief Function executed on Radio Rx Error event
 */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
void SubghzApp_Rx(void);
void SubghzApp_Tx(void);
void SubghzApp_Tx_Event(void);
void SubghzApp_ValueError(uint8_t errors);
/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
	/* USER CODE BEGIN SubghzApp_Init_1 */
	memcpy(buffer, "SAGV-CRRFD0000TTTTTT", 20);
	// Set a safe default threshold to avoid uninitialized use
	seuil = 10;
	// Initialize current page to a sane default until 0x66 confirms
	currentPage = 0;
	/* USER CODE END SubghzApp_Init_1 */

	/* Radio initialization */
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);

	/* USER CODE BEGIN SubghzApp_Init_2 */
	SUBGRF_SetTcxoMode(TCXO_CTRL_3_3V, 320);
	SUBGRF_SetStandby(STDBY_XOSC);

	CalibrationParams_t calib = {{1, 1, 1, 1, 1, 1, 1}};
	SUBGRF_Calibrate(calib);
	SUBGRF_CalibrateImage(APP_FREQUENCY);

	Radio.SetModem(APP_MODEM);
	Radio.SetChannel(APP_FREQUENCY);
	Radio.SetTxConfig(APP_MODEM, APP_OUTPUT_POWER, 0, APP_BANDWIDTH,
			APP_SPREADING_FACTOR, APP_CODING_RATE, APP_PREAMBLE_LENGTH,
			APP_HEADER, APP_CRC, 0, 0, 0, 500);
	Radio.SetRxConfig(APP_MODEM, APP_BANDWIDTH, APP_SPREADING_FACTOR,
			APP_CODING_RATE, 0, APP_PREAMBLE_LENGTH, APP_SYMB_TIMEOUT,
			APP_HEADER, APP_PAYLOAD_LENGTH, APP_CRC, 0, 0, 0, true);
	Radio.SetMaxPayloadLength(APP_MODEM, APP_PAYLOAD_LENGTH);

	HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

	if(!HAL_GPIO_ReadPin(MASTER_GPIO_Port, MASTER_Pin)) SubghzApp_Mode = SUBGHZ_APP_SLAVE;

	// Initialize sender_id from mode
	if(SubghzApp_Mode == SUBGHZ_APP_MASTER_1) sender_id = 1;
	else if(SubghzApp_Mode == SUBGHZ_APP_MASTER_2) sender_id = 2;
	else sender_id = 0;

	SubghzApp_Rx();

	if(SubghzApp_Mode == SUBGHZ_APP_MASTER_1)
	{
		HAL_ADCEx_Calibration_Start(&hadc);
		HAL_ADC_Start(&hadc);
	}
	/* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
	/* USER CODE BEGIN OnTxDone */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
	// Always go back to RX after a TX completes
	SubghzApp_Rx();
	/* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
	/* USER CODE BEGIN OnRxDone */
	uint8_t isError = 0;
	// Sanity checks to avoid overflow and underlength frames
	if (payload == NULL) {
		return;
	}
	if (size < sizeof(SUBGHZ_APP_ID)) {
		// Not even the ID, ignore
		return;
	}
	// Require minimum frame size for field parsing (indices up to 19)
	if (size < 20) {
		return;
	}
	// Bound the copy to our buffer size
	uint16_t copyLen = (size <= sizeof(RxBuffer)) ? size : sizeof(RxBuffer);
	memcpy(RxBuffer, payload, copyLen);
	if(memcmp(RxBuffer, SUBGHZ_APP_ID, sizeof(SUBGHZ_APP_ID)) == 0)
	{
		// Byte 9 : identité de l'émetteur (0, 1 ou 2)
		if(RxBuffer[9] == 0)
		{
			// Trame du récepteur (slave), ignorer ou traiter si besoin
		}
		else if(RxBuffer[9] == 1)
		{
			// Trame de l'émetteur 1
			// Indices: [11]=CASIER, [12]=GAZ, [13]=PRESSION
			if(RxBuffer[12] != 0)
			{
				NEXTION_SetText("main.t_gaz", "NOK");
				NEXTION_SetBackgroundColor("main.t_gaz", 64333);
				NEXTION_SetPicture("main.p_gaz", 2);

				isError |= 0b0100;
			}
			else
			{
				NEXTION_SetText("main.t_gaz", "OK");
				NEXTION_SetBackgroundColor("main.t_gaz", 5683);
				NEXTION_SetPicture("main.p_gaz", 3);
			}

			if(RxBuffer[11] != 0)
			{
				NEXTION_SetText("main.t_casier", "NOK");
				NEXTION_SetBackgroundColor("main.t_casier", 64333);
				NEXTION_SetPicture("main.p_casier", 4);

				isError |= 0b0010;
			}
			else
			{
				NEXTION_SetText("main.t_casier", "OK");
				NEXTION_SetBackgroundColor("main.t_casier", 5683);
				NEXTION_SetPicture("main.p_casier", 5);
			}

			// Pressure value is byte [13]
			NEXTION_SetValue("main.x_prs", RxBuffer[13]);
			if(RxBuffer[13] < seuil)
			{
				NEXTION_SetBackgroundColor("main.x_prs", 64333);
				NEXTION_SetPicture("main.p_prs", 0);

				isError |= 0b0001;
			}
			else
			{
				NEXTION_SetBackgroundColor("main.x_prs", 5683);
				NEXTION_SetPicture("main.p_prs", 1);
			}
		}
		else if(RxBuffer[9] == 2)
		{
			// Trame de l'émetteur 2
			// Index for secheur is [12]
			if(RxBuffer[12] != 0)
			{
				NEXTION_SetText("main.t_sec", "NOK");
				NEXTION_SetBackgroundColor("main.t_sec", 64333);
				NEXTION_SetPicture("main.p_sec", 6);

				isError |= 0b1000;
			}
			else
			{
				NEXTION_SetText("main.t_sec", "OK");
				NEXTION_SetBackgroundColor("main.t_sec", 5683);
				NEXTION_SetPicture("main.p_sec", 7);
			}
		}
		else
		{
			// Valeur inconnue, erreur ou trame non reconnue
			OnRxError();
			return;
		}

		TIM16->CNT = 0;

		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);

		NEXTION_SetText("main.t_radio", "OK");
		NEXTION_SetPicture("main.p_radio", 9);
		NEXTION_SetBackgroundColor("main.t_radio", 5683);

		// Display temperatures from received payload bytes [14..19]
		NEXTION_SetValue("boxState.x_e1T", (int)TMP275_ConvertRawToTemperature((int16_t)(RxBuffer[14] << 8 | RxBuffer[15]), tmp_config.resolution));
		NEXTION_SetValue("boxState.x_e1C", (int)TMP275_ConvertRawToTemperature((int16_t)(RxBuffer[16] << 8 | RxBuffer[17]), tmp_config.resolution));
		NEXTION_SetValue("boxState.x_e1A", (int)TMP275_ConvertRawToTemperature((int16_t)(RxBuffer[18] << 8 | RxBuffer[19]), tmp_config.resolution));

		if(isError != 0)
		{
			NEXTION_SendCommand("main.bcg_error.en=1");
			SubghzApp_ValueError(isError);
		}
		else NEXTION_SendCommand("main.bcg_error.en=0");
	}
	// After handling a frame, resume RX continuously
	SubghzApp_Rx();
	/* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
	/* USER CODE BEGIN OnTxTimeout */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
	/* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
	/* USER CODE BEGIN OnRxTimeout */
	// Resume RX after a timeout
	SubghzApp_Rx();
	/* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
	/* USER CODE BEGIN OnRxError */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
	// Resume RX after an error
	SubghzApp_Rx();
	/* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
void SubghzApp_Timeout(void)
{
	//TIM16 Event
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);

	NEXTION_SetText("main.t_radio", "DC");
	NEXTION_SetBackgroundColor("main.t_radio", 64333);
	// Fix: set radio picture, not secheur
	NEXTION_SendCommand("main.p_radio.pic=10");

	NEXTION_SetText("main.t_sec", "DC");
	NEXTION_SetBackgroundColor("main.t_sec", 64333);
	NEXTION_SendCommand("main.p_sec.pic=6");

	NEXTION_SetText("main.t_gaz", "DC");
	NEXTION_SetBackgroundColor("main.t_gaz", 64333);
	NEXTION_SendCommand("main.p_gaz.pic=2");

	NEXTION_SetText("main.t_casier", "DC");
	NEXTION_SetBackgroundColor("main.t_casier", 64333);
	NEXTION_SendCommand("main.p_casier.pic=4");

	NEXTION_SetBackgroundColor("main.x_prs", 64333);
	NEXTION_SendCommand("main.p_prs.pic=0");

	NEXTION_SendCommand("main.bcg_error.en=1");

	SubghzApp_ValueError((uint8_t)0b10000);
}

void SubghzApp_Event(void)
{
	// TIM2 Event (configured for 5 seconds)
	if(SubghzApp_Mode == SUBGHZ_APP_SLAVE)
	{
		// SLAVE only receives, no BEACON, just keep RX
		SubghzApp_Rx();
	}
	else if(SubghzApp_Mode == SUBGHZ_APP_MASTER_1)
	{
		// MASTER_1 transmits every period
		SubghzApp_Tx_Event();
	}
	else if(SubghzApp_Mode == SUBGHZ_APP_MASTER_2)
	{
		// MASTER_2 transmits every period
		SubghzApp_Tx_Event();
	}

	// Maintain legacy state counter but it's no longer used for scheduling
	SubghzApp_State_n++;
	if(SubghzApp_State_n > 5) SubghzApp_State_n = 0;
}

void SubghzApp_Process(void)
{
	float tmp1;
	float tmp2;
	float tmp3;

	NEXTION_SetValue("main.mcu_on", 1);
	NEXTION_SendCommand("main.bcg_error.en=0");
	NEXTION_SetBackgroundColor("main", 65438);
	NEXTION_SetBackgroundColor("main.t_mcu", 5683);
	NEXTION_SendCommand("main.p_mcu.pic=13");

	TMP275_ReadTemperature(&tmp_sensor_1, &tmp1);
	TMP275_ReadTemperature(&tmp_sensor_2, &tmp2);
	TMP275_ReadTemperature(&tmp_sensor_3, &tmp3);

	NEXTION_SetValue("boxState.x_r1T", (int)(tmp1 * 10));
	NEXTION_SetValue("boxState.x_r1C", (int)(tmp2 * 10));
	NEXTION_SetValue("boxState.x_r1A", (int)(tmp3 * 10));
}

void SubghzApp_Rx(void)
{
	if(SubghzApp_Mode == SUBGHZ_APP_MASTER_1) SubghzApp_State = SUBGHZ_APP_MASTER_1_RX;
	else if(SubghzApp_Mode == SUBGHZ_APP_MASTER_2) SubghzApp_State = SUBGHZ_APP_MASTER_2_RX;
	else if(SubghzApp_Mode == SUBGHZ_APP_SLAVE) SubghzApp_State = SUBGHZ_APP_SLAVE_RX;


	SUBGRF_SetSwitch(RFO_HP, RFSWITCH_RX);

	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 1);
	// Do not force LED_GREEN off here; preserve TX indication until next event
	// HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);

	Radio.Rx(0);
}

void SubghzApp_Tx(void)
{
	// Byte 9 : identité de l'émetteur (0, 1 ou 2)
	buffer[9] = sender_id;
	if(sender_id == 1) SubghzApp_State = SUBGHZ_APP_MASTER_1_TX;
	else if(sender_id == 2) SubghzApp_State = SUBGHZ_APP_MASTER_2_TX;
	else if(sender_id == 0) SubghzApp_State = SUBGHZ_APP_SLAVE_TX;

	// Indicate TX: Green ON, Blue OFF
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

	Radio.Send(buffer, sizeof(buffer));
}

void SubghzApp_Tx_Event(void)
{
	int16_t tmp1;
	int16_t tmp2;
	int16_t tmp3;

	TMP275_ReadTemperatureRaw(&tmp_sensor_1, &tmp1);
	TMP275_ReadTemperatureRaw(&tmp_sensor_2, &tmp2);
	TMP275_ReadTemperatureRaw(&tmp_sensor_3, &tmp3);

	buffer[14] = (uint8_t) (tmp1 >> 8);
	buffer[15] = (uint8_t) tmp1;
	buffer[16] = (uint8_t) (tmp2 >> 8);
	buffer[17] = (uint8_t) tmp2;
	buffer[18] = (uint8_t) (tmp3 >> 8);
	buffer[19] = (uint8_t) tmp3;

	if(SubghzApp_Mode == SUBGHZ_APP_MASTER_1)
	{
		SubghzApp_State = SUBGHZ_APP_MASTER_1_TX;

		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 20);

		buffer[10] = 0;
		buffer[11] = HAL_GPIO_ReadPin(CASIER_GPIO_Port, CASIER_Pin);
		buffer[12] = HAL_GPIO_ReadPin(GAZ_GPIO_Port, GAZ_Pin);
		buffer[13] = (((float)(HAL_ADC_GetValue(&hadc) * 10.3421) / 65520) * 10);
		HAL_ADC_Stop(&hadc);
	}
	else if (SubghzApp_Mode == SUBGHZ_APP_MASTER_2)
	{
		SubghzApp_State = SUBGHZ_APP_MASTER_2_TX;
		buffer[12] = HAL_GPIO_ReadPin(SECHEUR_GPIO_Port, SECHEUR_Pin);
	}

	// Schedule random jitter before transmission (0-500 ms) without blocking in ISR
	int jitter = rand() % 501;
	tx_due_tick = HAL_GetTick() + (uint32_t)jitter;
	tx_jitter_pending = 1;
}

void SubghzApp_BackgroundProcess(void)
{
	if (tx_jitter_pending)
	{
		uint32_t now = HAL_GetTick();
		// Handle tick wrap-around correctly using subtraction
		if ((int32_t)(now - tx_due_tick) >= 0)
		{
			// Clear pending first to avoid re-entry
			tx_jitter_pending = 0;
			SubghzApp_Tx();
		}
	}
}

void SubghzApp_UART_RxDone(uint8_t *rxBuffer, uint16_t rxLen)
{
	// Validate input parameters
	if (rxBuffer == NULL || rxLen == 0) {
		return;
	}
	
	// Handle standard Nextion reports (optional)
	// 0x66: Current page ID -> 2 bytes + 3 terminators => len = 5
	if (rxLen == 5 && rxBuffer[0] == 0x66)
	{
		currentPage = rxBuffer[1];
		return;
	}

	// Custom protocol
	// 0x01 <page> 0xFF 0xFF 0xFF => len must be exactly 5
	if (rxLen == 5 && rxBuffer[0] == 0x01)
	{
		uint8_t requestedPage = rxBuffer[1];
		if(requestedPage != currentPage)
		{
			NEXTION_ChangePage(requestedPage);
			currentPage = requestedPage;
		}
		return;
	}

	if(rxLen >= 5 && rxBuffer[0] == 0x02)
	{
		// Réglage seuil défaut
		seuil = rxBuffer[1];
		return;
	}
	if(rxLen >= 5 && rxBuffer[0] == 0x03)
	{
		// Mise à jour OTA
		return;
	}
	if(rxLen >= 5 && rxBuffer[0] == 0x04)
	{
		// Test Radio
		return;
	}
	if(rxLen >= 5 && rxBuffer[0] == 0x05)
	{
		// Test HMI
		return;
	}
	if(rxLen == 5 && rxBuffer[0] == 0x07)
	{
		// Non utilisé
		return;
	}
	if(rxLen >= 5 && rxBuffer[0] == 0x71)
	{
		// Resultat de l'opération "GET"
		// Verify pointer is valid before copying
		if (redeem_USART_ptr != NULL && redeem_USART_length > 0 && 
		    redeem_USART_length <= (int)(rxLen - 1)) {
			memcpy(redeem_USART_ptr, rxBuffer + 1, redeem_USART_length);
			redeem_done = 1;
		}
		return;
	}
}

void SubghzApp_Start(void)
{
	// No BEACON anymore. Ensure RX is set for SLAVE and timers are handled in main.
	if(SubghzApp_Mode == SUBGHZ_APP_SLAVE)
	{
		SubghzApp_Rx();
	}
}

void SubghzApp_ValueError(uint8_t errors)
{
	if(errors != past_errors)
	{
		NEXTION_SendCommand("history.t6.txt=history.t5.txt");
		NEXTION_SendCommand("history.t5.txt=history.t4.txt");
		NEXTION_SendCommand("history.t4.txt=history.t3.txt");
		NEXTION_SendCommand("history.t3.txt=history.t2.txt");
		NEXTION_SendCommand("history.t2.txt=history.t1.txt");

		NEXTION_SendCommand("covx rtc2,history.buffer.txt,0,0");
		NEXTION_SendCommand("history.t1.txt=\"Le \"+history.buffer.txt+\"/\"");
		NEXTION_SendCommand("covx rtc1,history.buffer.txt,0,0");
		NEXTION_SendCommand("history.t1.txt+=history.buffer.txt+\" a \"");
		NEXTION_SendCommand("covx rtc3,history.buffer.txt,0,0");
		NEXTION_SendCommand("history.t1.txt+=history.buffer.txt+\":\"");
		NEXTION_SendCommand("covx rtc4,history.buffer.txt,0,0");
		NEXTION_SendCommand("history.t1.txt+=history.buffer.txt+\":\"");
		NEXTION_SendCommand("covx rtc5,history.buffer.txt,0,0");
		NEXTION_SendCommand("history.t1.txt+=history.buffer.txt+\" : \"");

		if(errors >> 4 == 1 && past_errors >> 4 == 0)
		{
			// Alarme radio
			NEXTION_SendCommand("history.t1.txt+=\"radio,\"");
		}
		if(errors >> 3 == 1 && past_errors >> 3 == 0)
		{
			// Alarme secheur
			NEXTION_SendCommand("history.t1.txt+=\"secheur,\"");
		}
		if(errors >> 2 == 1 && past_errors >> 2 == 0)
		{
			// Alarme gaz
			NEXTION_SendCommand("history.t1.txt+=\"gaz,\"");
		}
		if(errors >> 1 == 1 && past_errors >> 1 == 0)
		{
			// Alarme casier
			NEXTION_SendCommand("history.t1.txt+=\"casier,\"");
		}
		if(errors >> 0 == 1 && past_errors >> 0 == 0)
		{
			// Alarme pression
			NEXTION_SendCommand("history.t1.txt+=\"pression,\"");
		}
		NEXTION_SendCommand("history.t1.txt-=1");

		past_errors = errors;
	}
}
/* USER CODE END PrFD */