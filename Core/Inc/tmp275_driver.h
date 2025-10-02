/**
 ******************************************************************************
 * @file    tmp275_driver.h
 * @author  Kenzo Gallauziaux
 * @brief   TMP275AQDGKRQ1 Temperature Sensor Library Header
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Library for TMP275AQDGKRQ1 temperature sensor on STM32-WL55
 * Communication: I2C
 * Resolution: 9 to 12 bits
 * Accuracy: ±0.5°C (typ) from -25°C to +85°C
 *
 ******************************************************************************
 */

#ifndef __TMP275_DRIVER_H
#define __TMP275_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ===== INCLUDES ===== */
#include "stm32wlxx.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== DÉFINITIONS PUBLIQUES ===== */

// Codes de retour
typedef enum {
	TMP275_OK       		= 0x00,
	TMP275_ERROR    		= 0x01,
	TMP275_TIMEOUT  		= 0x02,
	TMP275_BUSY     		= 0x03,
	TMP275_NACK				= 0x04,
	TMP275_BUS_ERROR		= 0x05,
	TMP275_INVALID_PARAM	= 0x06,
} TMP275_Status_t;

// Résolution de conversion
typedef enum {
	TMP275_RES_9BIT  = 0x00,   // 0.5°C
	TMP275_RES_10BIT = 0x01,   // 0.25°C
	TMP275_RES_11BIT = 0x02,   // 0.125°C
	TMP275_RES_12BIT = 0x03    // 0.0625°C (par défaut)
} TMP275_Resolution_t;

// Mode de fonctionnement
typedef enum {
	TMP275_MODE_ACTIVE 		= 	0x00,
	TMP275_MODE_SHUTDOWN 	= 	0x01
} TMP275_Mode_t;

// Configuration de l'alarme
typedef enum {
	TMP275_COMPARATOR_MODE = 0x00,
	TMP275_INTERRUPT_MODE  = 0x01
} TMP275_AlertMode_t;

// Structure de configuration
typedef struct {
	TMP275_Resolution_t resolution;
	TMP275_Mode_t mode;
	TMP275_AlertMode_t alert_mode;
	uint8_t fault_queue;        // 1, 2, 4, or 6 consecutive faults
	bool alert_polarity_high;   // true = active high, false = active low
	float temp_low_threshold;   // Seuil bas en °C
	float temp_high_threshold;  // Seuil haut en °C
} TMP275_Config_t;

// Structure de données du capteur
typedef struct {
	I2C_TypeDef* i2c_instance;
	uint8_t device_address;
	TMP275_Config_t config;
	bool initialized;
} TMP275_Handle_t;

extern TMP275_Handle_t tmp_sensor_1;
extern TMP275_Handle_t tmp_sensor_2;
extern TMP275_Handle_t tmp_sensor_3;
extern const TMP275_Config_t tmp_config;

/* ===== PROTOTYPES DES FONCTIONS PUBLIQUES ===== */

// Initialisation et configuration
TMP275_Status_t TMP275_Init(TMP275_Handle_t* htmp275);
TMP275_Status_t TMP275_DeInit(TMP275_Handle_t* htmp275);
TMP275_Status_t TMP275_Configure(TMP275_Handle_t* htmp275, TMP275_Config_t* config);

// Opérations de lecture
TMP275_Status_t TMP275_ReadTemperature(TMP275_Handle_t* htmp275, float* temperature);
TMP275_Status_t TMP275_ReadTemperatureRaw(TMP275_Handle_t* htmp275, int16_t* temp_raw);
TMP275_Status_t TMP275_ReadConfiguration(TMP275_Handle_t* htmp275, uint16_t* config);

// Gestion des seuils d'alarme
TMP275_Status_t TMP275_SetLowThreshold(TMP275_Handle_t* htmp275, float temperature);
TMP275_Status_t TMP275_SetHighThreshold(TMP275_Handle_t* htmp275, float temperature);
TMP275_Status_t TMP275_GetLowThreshold(TMP275_Handle_t* htmp275, float* temperature);
TMP275_Status_t TMP275_GetHighThreshold(TMP275_Handle_t* htmp275, float* temperature);

// Contrôle du mode
TMP275_Status_t TMP275_SetMode(TMP275_Handle_t* htmp275, TMP275_Mode_t mode);
TMP275_Status_t TMP275_SetResolution(TMP275_Handle_t* htmp275, TMP275_Resolution_t resolution);

// Utilitaires
bool TMP275_IsInitialized(TMP275_Handle_t* htmp275);
TMP275_Status_t TMP275_SoftReset(TMP275_Handle_t* htmp275);
float TMP275_ConvertRawToTemperature(int16_t raw_temp, TMP275_Resolution_t resolution);
int16_t TMP275_ConvertTemperatureToRaw(float temperature, TMP275_Resolution_t resolution);

// Configuration I2C (fonctions d'aide)
void TMP275_I2C_Init(void);
void TMP275_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TMP275_DRIVER_H */

/************************ (C) COPYRIGHT Kenzo Gallauziaux *****END OF FILE****/
