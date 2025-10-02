/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERROR_MANAGER_H
#define __ERROR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define SUBGHZSPI_INIT_RADIO_BUSY						0x30
#define SUBGHZSPI_INIT_TIMEOUT							0x31
#define SUBGHZSPI_RECEIVE_TIMEOUT						0x32

#define SUBGHZ_SEND_COMMAND_OVERFLOW					0x41
#define SUBGHZ_WRITE_BUFFER_OVERFLOW					0x42
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Manager(uint8_t errorCode);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_MANAGER_H */
