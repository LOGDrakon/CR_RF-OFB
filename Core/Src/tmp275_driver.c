/**
 ******************************************************************************
 * @file    tmp275_driver.c
 * @author  Kenzo Gallauziaux
 * @brief   TMP275AQDGKRQ1 Temperature Sensor Library Implementation
 * @date    2025
 ******************************************************************************
 */

/* ===== INCLUDES ===== */
#include "tmp275_driver.h"

/* ===== DÉFINITIONS PRIVÉES ===== */

const TMP275_Config_t tmp_config = {
		.alert_mode = TMP275_COMPARATOR_MODE,
		.alert_polarity_high = false,
		.fault_queue = 1,
		.mode = TMP275_MODE_ACTIVE,
		.resolution = TMP275_RES_12BIT,
		.temp_high_threshold = 35.0f,
		.temp_low_threshold = 30.0f
};

TMP275_Handle_t tmp_sensor_1 = {
		.device_address = 0x48,
		.i2c_instance = I2C1,
		.config = tmp_config
};
TMP275_Handle_t tmp_sensor_2 = {
		.device_address = 0x4A,
		.i2c_instance = I2C1,
		.config = tmp_config
};
TMP275_Handle_t tmp_sensor_3 = {
		.device_address = 0x49,
		.i2c_instance = I2C1,
		.config = tmp_config
};

// Define I2C utils
#define I2C_TIMING_32MHZ_100K    0x00707CBB

// Adresses des registres TMP275
#define TMP275_REG_TEMP         0x00
#define TMP275_REG_CONFIG       0x01
#define TMP275_REG_TLOW         0x02
#define TMP275_REG_THIGH        0x03

// Bits de configuration
#define TMP275_CONFIG_SD_BIT    0x0001  // Shutdown
#define TMP275_CONFIG_TM_BIT    0x0002  // Thermostat mode
#define TMP275_CONFIG_POL_BIT   0x0004  // Alert polarity
#define TMP275_CONFIG_F0_BIT    0x0008  // Fault queue bit 0
#define TMP275_CONFIG_F1_BIT    0x0010  // Fault queue bit 1
#define TMP275_CONFIG_R0_BIT    0x0020  // Resolution bit 0
#define TMP275_CONFIG_R1_BIT    0x0040  // Resolution bit 1
#define TMP275_CONFIG_OS_BIT    0x8000  // One-shot

// Timeouts
#define TMP275_TIMEOUT_MS       1000
#define TMP275_I2C_TIMEOUT      100000

// Facteurs de conversion
static const float resolution_factors[] = {
		0.5f,     // 9-bit
		0.25f,    // 10-bit
		0.125f,   // 11-bit
		0.0625f   // 12-bit
};

/* ===== FONCTIONS PRIVÉES ===== */
static TMP275_Status_t I2C_WriteRegister(TMP275_Handle_t* htmp275, uint8_t reg, uint8_t *data, uint8_t length)
{
	I2C_TypeDef* I2Cx = htmp275->i2c_instance;
	uint8_t device_address = htmp275->device_address << 1;

	while (I2Cx->ISR & I2C_ISR_BUSY);

	I2Cx->CR2 = 0;
	I2Cx->CR2 &= ~I2C_CR2_ADD10;
	I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
	I2Cx->CR2 |= ((length+1) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
	I2Cx->CR2 &= ~I2C_CR2_RD_WRN;
	I2Cx->CR2 |= I2C_CR2_START;

	//wait TXIS flag
	while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
		//check errors
		if (I2Cx->ISR & I2C_ISR_ARLO)
		{
			return TMP275_BUS_ERROR;
		}
		else if (I2Cx->ISR & I2C_ISR_NACKF)
		{
			return TMP275_NACK;
		}
	}
	//write data
	I2Cx->TXDR = reg;

	//send data
	for (uint16_t i = 0; i < length; i++) {
		//wait TXIS flag
		while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
			//check errors
			if (I2Cx->ISR & I2C_ISR_ARLO)
			{
				return TMP275_BUS_ERROR;
			}
			else if (I2Cx->ISR & I2C_ISR_NACKF)
			{
				return TMP275_NACK;
			}
		}
		//write data
		I2Cx->TXDR = data[i];
	}

	//wait end of transmission
	while (!(I2Cx->ISR & I2C_ISR_TC)) {
		if (I2Cx->ISR & I2C_ISR_ARLO)
		{
			return TMP275_BUS_ERROR;
		}
		else if (I2Cx->ISR & I2C_ISR_NACKF)
		{
			return TMP275_NACK;
		}
	}
	//generate stop
	I2Cx->CR2 |= I2C_CR2_STOP;
	//wait stop
	while (!(I2Cx->ISR & I2C_ISR_STOPF));
	while(I2Cx->ISR & I2C_ISR_BUSY);
	return TMP275_OK;
}

static TMP275_Status_t I2C_ReadRegister(TMP275_Handle_t* htmp275, uint8_t reg, uint8_t *buffer, uint8_t length)
{
	I2C_TypeDef* I2Cx = htmp275->i2c_instance;
	uint8_t device_address = htmp275->device_address << 1;

	while (I2Cx->ISR & I2C_ISR_BUSY);

	//set reg into pointer register
	I2Cx->CR2 = 0;
	I2Cx->CR2 &= ~I2C_CR2_ADD10;
	I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
	I2Cx->CR2 |= (1 << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
	I2Cx->CR2 &= ~I2C_CR2_RD_WRN;
	I2Cx->CR2 |= I2C_CR2_START;

	//wait TXIS flag
	while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
		//check errors
		if (I2Cx->ISR & I2C_ISR_ARLO)
		{
			return TMP275_BUS_ERROR;
		}
		else if (I2Cx->ISR & I2C_ISR_NACKF)
		{
			return TMP275_NACK;
		}
	}
	//write reg
	I2Cx->TXDR = reg;

	// Attendre la fin de la première transmission
	while (!(I2Cx->ISR & I2C_ISR_TC)) {
	    if (I2Cx->ISR & I2C_ISR_ARLO) return TMP275_BUS_ERROR;
	    if (I2Cx->ISR & I2C_ISR_NACKF) return TMP275_NACK;
	}

	// Reconfigurer pour la lecture
	I2Cx->CR2 = 0; // Reset complet
	I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
	I2Cx->CR2 |= (length << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
	I2Cx->CR2 |= I2C_CR2_RD_WRN;
	I2Cx->CR2 |= I2C_CR2_START;

	//receive data
	for (uint16_t i = 0; i < length; i++) {
		//wait for data
		while (!(I2Cx->ISR & I2C_ISR_RXNE)) {
			//check errors
			if (I2Cx->ISR & I2C_ISR_ARLO)
			{
				return TMP275_BUS_ERROR;
			}
			else if (I2Cx->ISR & I2C_ISR_NACKF)
			{
				return TMP275_NACK;
			}
		}
		//read data
		buffer[i] = I2Cx->RXDR;
	}

	//generate stop
	I2Cx->CR2 |= I2C_CR2_STOP;
	//wait stop
	while (!(I2Cx->ISR & I2C_ISR_STOPF));
	while(I2Cx->ISR & I2C_ISR_BUSY);
	return TMP275_OK;
}

float TMP275_ConvertRawToTemperature(int16_t raw_temp, TMP275_Resolution_t resolution) {
	return (float)raw_temp * resolution_factors[resolution];
}

int16_t TMP275_ConvertTemperatureToRaw(float temperature, TMP275_Resolution_t resolution) {
	return (int16_t)(temperature / resolution_factors[resolution]);
}

/* ===== IMPLÉMENTATION DES FONCTIONS PUBLIQUES ===== */

TMP275_Status_t TMP275_Init(TMP275_Handle_t* htmp275) {
	if (htmp275 == NULL) {
		return TMP275_ERROR;
	}

	htmp275->initialized = false;

	// Tester la communication
	uint8_t config_reg;
	if (I2C_ReadRegister(htmp275, TMP275_REG_CONFIG, &config_reg, 1) != TMP275_OK) {
		return TMP275_ERROR;
	}

	// Appliquer la configuration par défaut
	if (TMP275_Configure(htmp275, &htmp275->config) != TMP275_OK) {
		return TMP275_ERROR;
	}

	htmp275->initialized = true;
	return TMP275_OK;
}

TMP275_Status_t TMP275_DeInit(TMP275_Handle_t* htmp275) {
	if (htmp275 == NULL) {
		return TMP275_ERROR;
	}

	// Mettre en mode shutdown
	TMP275_SetMode(htmp275, TMP275_MODE_SHUTDOWN);

	htmp275->initialized = false;
	return TMP275_OK;
}

TMP275_Status_t TMP275_Configure(TMP275_Handle_t* htmp275, TMP275_Config_t* config) {
	if (htmp275 == NULL || config == NULL) {
		return TMP275_ERROR;
	}

	uint8_t config_reg = 0;

	// Construire le registre de configuration
	if (config->mode == TMP275_MODE_SHUTDOWN) {
		config_reg |= TMP275_CONFIG_SD_BIT;
	}

	if (config->alert_mode == TMP275_INTERRUPT_MODE) {
		config_reg |= TMP275_CONFIG_TM_BIT;
	}

	if (config->alert_polarity_high) {
		config_reg |= TMP275_CONFIG_POL_BIT;
	}

	// Configuration de la queue de fautes
	switch (config->fault_queue) {
	case 2: config_reg |= TMP275_CONFIG_F0_BIT; break;
	case 4: config_reg |= TMP275_CONFIG_F1_BIT; break;
	case 6: config_reg |= TMP275_CONFIG_F1_BIT | TMP275_CONFIG_F0_BIT; break;
	default: break; // 1 fault (par défaut)
	}

	// Configuration de la résolution
	config_reg |= config->resolution << 5;

	// Écrire la configuration
	if (I2C_WriteRegister(htmp275, TMP275_REG_CONFIG, &config_reg, 1) != TMP275_OK) {
		return TMP275_ERROR;
	}

	// Configurer les seuils
	int16_t raw_temp = TMP275_ConvertTemperatureToRaw(htmp275->config.temp_low_threshold, htmp275->config.resolution);
	if (I2C_WriteRegister(htmp275, TMP275_REG_TLOW, (uint8_t[]){raw_temp >> 8, raw_temp}, 2) != TMP275_OK) {
		return TMP275_ERROR;
	}

	raw_temp = TMP275_ConvertTemperatureToRaw(htmp275->config.temp_high_threshold, htmp275->config.resolution);
	if (I2C_WriteRegister(htmp275, TMP275_REG_THIGH, (uint8_t[]){raw_temp >> 8, raw_temp}, 2) != TMP275_OK) {
		return TMP275_ERROR;
	}

	// Sauvegarder la configuration
	htmp275->config = *config;

	return TMP275_OK;
}

TMP275_Status_t TMP275_ReadTemperature(TMP275_Handle_t* htmp275, float* temperature) {
	if (htmp275 == NULL || temperature == NULL || !htmp275->initialized) {
		return TMP275_ERROR;
	}

	int16_t raw_temp;
	TMP275_Status_t status = TMP275_ReadTemperatureRaw(htmp275, &raw_temp);

	if (status == TMP275_OK) {
		*temperature = TMP275_ConvertRawToTemperature(raw_temp, htmp275->config.resolution);
	}

	return status;
}

TMP275_Status_t TMP275_ReadTemperatureRaw(TMP275_Handle_t* htmp275, int16_t* temp_raw) {
	if (htmp275 == NULL || temp_raw == NULL || !htmp275->initialized) {
		return TMP275_ERROR;
	}

	uint8_t _raw[2];
	TMP275_Status_t status = I2C_ReadRegister(htmp275, TMP275_REG_TEMP, _raw, 2);

	if (status != TMP275_OK) *temp_raw = 0;
	else *temp_raw = (int16_t)((_raw[0] << 8) | _raw[1]) >> 4;

	return status;
}

TMP275_Status_t TMP275_SetLowThreshold(TMP275_Handle_t* htmp275, float temperature) {
	if (htmp275 == NULL || !htmp275->initialized) {
		return TMP275_ERROR;
	}

	int16_t raw_temp = TMP275_ConvertTemperatureToRaw(temperature, htmp275->config.resolution);
	TMP275_Status_t status = I2C_WriteRegister(htmp275, TMP275_REG_TLOW, (uint8_t[]){raw_temp >> 8, raw_temp}, 2);

	if (status == TMP275_OK) {
		htmp275->config.temp_low_threshold = temperature;
	}

	return status;
}

TMP275_Status_t TMP275_SetHighThreshold(TMP275_Handle_t* htmp275, float temperature) {
	if (htmp275 == NULL || !htmp275->initialized) {
		return TMP275_ERROR;
	}

	int16_t raw_temp = TMP275_ConvertTemperatureToRaw(temperature, htmp275->config.resolution);
	TMP275_Status_t status = I2C_WriteRegister(htmp275, TMP275_REG_THIGH, (uint8_t[]){raw_temp >> 8, raw_temp}, 2);

	if (status == TMP275_OK) {
		htmp275->config.temp_high_threshold = temperature;
	}

	return status;
}

TMP275_Status_t TMP275_SetMode(TMP275_Handle_t* htmp275, TMP275_Mode_t mode) {
	if (htmp275 == NULL || !htmp275->initialized) {
		return TMP275_ERROR;
	}

	htmp275->config.mode = mode;
	return TMP275_Configure(htmp275, &htmp275->config);
}

bool TMP275_IsInitialized(TMP275_Handle_t* htmp275) {
	return (htmp275 != NULL) && htmp275->initialized;
}

void TMP275_I2C_Init(void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

	I2C1->CR1 &= ~I2C_CR1_PE;

	I2C1->TIMINGR = I2C_TIMING_32MHZ_100K;

	I2C1->CR1 |= I2C_CR1_PE;
}

void TMP275_GPIO_Init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Configuration PB6 (SCL) et PB7 (SDA) pour I2C1
	// Mode alternatif
	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
	// Open-drain
	GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
	// Vitesse élevée
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
	// Pull-up interne (optionnel si pull-up externes présentes)
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
	// Fonction alternative AF4 pour I2C1
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	GPIOB->AFR[0] |= ((0x4 << GPIO_AFRL_AFSEL6_Pos) | (0x4 << GPIO_AFRL_AFSEL7_Pos));
}

/************************ (C) COPYRIGHT Kenzo Gallauziaux *****END OF FILE****/
