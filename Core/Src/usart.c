/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
#include "subghz_phy_app.h"
#include "nextion_driver.h"

#define UART1_RX_BUFFER_MAX_SIZE 100

uint8_t uart1_rx_data;
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_MAX_SIZE];
uint8_t uart1_rx_index;
uint8_t waiting_for_ack;
uint8_t ack_code;

void NEXTION_WaitForACK(void);
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */
  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void USART1_Interrupt(void)
{
	if(uart1_rx_index == 0)
	{
		if(waiting_for_ack) ack_code = 0;
		memset(uart1_rx_buffer, 0, UART1_RX_BUFFER_MAX_SIZE);
	}

	if((uart1_rx_index + 1) > UART1_RX_BUFFER_MAX_SIZE)
	{
		//overflow
		uart1_rx_index = 0;
		waiting_for_ack = 0;
	}

	uart1_rx_buffer[uart1_rx_index] = uart1_rx_data;
	uart1_rx_index++;

	// Ensure we have at least 3 bytes before checking the 3x 0xFF terminator
	if (uart1_rx_index >= 3 &&
		uart1_rx_buffer[uart1_rx_index-1] == 0xFF &&
		uart1_rx_buffer[uart1_rx_index-2] == 0xFF &&
		uart1_rx_buffer[uart1_rx_index-3] == 0xFF)
	{
		//command received
		if(uart1_rx_index == 4 && waiting_for_ack)
		{
			//ACK (e.g., 0x01 0xFF 0xFF 0xFF)
			ack_code = uart1_rx_buffer[0];
			waiting_for_ack = 0;
		}
		else if(uart1_rx_index > 4)
		{
			//command with payload, forward to app with length for validation
			SubghzApp_UART_RxDone(uart1_rx_buffer, uart1_rx_index);
		}
		else
		{
			//error non-available command
		}

		// Optional: lightweight debug
		// for (uint8_t i = 0; i < uart1_rx_index; ++i) {
		//     ITM_SendChar(uart1_rx_buffer[i]);
		// }
		// ITM_SendChar('\n');

		uart1_rx_index = 0;
	}
	//relancer l'écoute
	HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
}

void NEXTION_SendCommand(const char* command)
{
	uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF};
	//send command
	HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), 10);
	//send 3 last bytes
	HAL_UART_Transmit(&huart1, end_cmd, 3, 10);

	NEXTION_WaitForACK();
}

void NEXTION_WaitForACK(void)
{
	uint8_t time = 0;
	waiting_for_ack = 1;
	while(time < 10)
	{
		if(ack_code != 0)
		{
			return;
		}
		HAL_Delay(1);
		time++;
	}
	ack_code = 0xFF;
}

void NEXTION_SetText(const char* object_name, const char* text)
{
	char command[128];
	sprintf(command, "%s.txt=\"%s\"", object_name, text);
	NEXTION_SendCommand(command);
}

void NEXTION_SetValue(const char* object_name, int value)
{
	char command[128];
	sprintf(command, "%s.val=%d", object_name, value);
	NEXTION_SendCommand(command);
}

void NEXTION_SetBackgroundColor(const char* object_name, uint16_t color)
{
	char command[128];
	sprintf(command, "%s.bco=%d", object_name, color);
	NEXTION_SendCommand(command);
}

void NEXTION_SetTextColor(const char* object_name, uint16_t color)
{
	char command[128];
	sprintf(command, "%s.pco=%d", object_name, color);
	NEXTION_SendCommand(command);
}

void NEXTION_ChangePage(uint8_t page_id)
{
	char command[128];
	sprintf(command, "page %d", page_id);
	NEXTION_SendCommand(command);
}

void NEXTION_SetVisible(const char* object_name, uint8_t visible)
{
	char command[128];
	sprintf(command, "%s.aph=%d", object_name, visible);
	NEXTION_SendCommand(command);
}

void NEXTION_GetValue(const char* object_name)
{
	char command[128];
	sprintf(command, "get %s.val", object_name);
	NEXTION_SendCommand(command);
}

void NEXTION_ClearScreen(uint16_t color)
{
	char command[128];
	sprintf(command, "cls %d", color);
	NEXTION_SendCommand(command);
}

void NEXTION_SetPicture(const char* object_name, uint8_t picture)
{
	char command[128];
	sprintf(command, "%s.pic=%d", object_name, picture);
	NEXTION_SendCommand(command);
}
/* USER CODE END 1 */