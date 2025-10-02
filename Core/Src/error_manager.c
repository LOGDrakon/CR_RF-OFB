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
	__disable_irq();
	switch(errorCode)
	{
	case SUBGHZSPI_INIT_RADIO_BUSY:
		// for radio busy in subghzspi init
	case SUBGHZSPI_INIT_TIMEOUT:
		// for timeout in subghspi init
	case SUBGHZ_WRITE_BUFFER_OVERFLOW:
		// for txLength too large in "SUBGHZ_Write_Buffer"
	case SUBGHZ_SEND_COMMAND_OVERFLOW:
		// for txLength too large in "SUBGHZ_SendCommand"
	default:
		// errorCode unknown
	}

	while(1);
}
