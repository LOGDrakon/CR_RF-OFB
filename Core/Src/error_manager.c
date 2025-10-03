/* Includes ------------------------------------------------------------------*/
#include "error_manager.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
void Error_Manager(uint8_t errorCode)
{
	// Disable interrupts to prevent system corruption
	__disable_irq();
	
	// Log error code (if debug interface available)
	#ifdef DEBUG
	printf("ERROR: Code 0x%02X\r\n", errorCode);
	#endif
	
	// Set error indication LEDs
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	
	switch(errorCode)
	{
	case SUBGHZSPI_INIT_RADIO_BUSY:
		// Radio busy during initialization - could retry after delay
		break;
	case SUBGHZSPI_INIT_TIMEOUT:
		// Timeout during radio SPI initialization
		break;
	case SUBGHZ_WRITE_BUFFER_OVERFLOW:
		// Buffer overflow in SUBGHZ_Write_Buffer - data corruption risk
		break;
	case SUBGHZ_SEND_COMMAND_OVERFLOW:
		// Command buffer overflow in SUBGHZ_SendCommand
		break;
	default:
		// Unknown error code - unexpected condition
		break;
	}
	
	// Store error in backup registers for post-mortem analysis
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, errorCode);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, HAL_GetTick());
	
	// System reset after error logging
	HAL_Delay(1000); // Allow time for external observer
	NVIC_SystemReset();
}
