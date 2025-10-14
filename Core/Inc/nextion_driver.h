/**
 ******************************************************************************
 * @file    nextion_driver.h
 * @brief   Nextion HMI Driver public interface
 * @date    2025
 ******************************************************************************
 */

#ifndef __NEXTION_DRIVER_H
#define __NEXTION_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"
#include <stdint.h>

/* Function Prototypes -------------------------------------------------------*/
void NEXTION_SendCommand(const char* command);
void NEXTION_SetText(const char* object_name, const char* text);
void NEXTION_SetValue(const char* object_name, int value);
void NEXTION_SetBackgroundColor(const char* object_name, uint32_t color);
void NEXTION_SetTextColor(const char* object_name, uint16_t color);
void NEXTION_ChangePage(uint8_t page_id);
void NEXTION_SetVisible(const char* object_name, uint8_t visible);
void NEXTION_GetValue(const char* object_name);
void NEXTION_ClearScreen(uint16_t color);
void NEXTION_SetPicture(const char* object_name, int pic_id);
void NEXTION_WaitForACK(void);

#ifdef __cplusplus
}
#endif

#endif /* __NEXTION_DRIVER_H */
