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
// Robust timeout helper that also works if HAL_GetTick() doesn't advance
typedef struct {
    uint32_t start_tick;
    uint32_t last_tick;
    uint32_t spin;
} timeout_ctx_t;

static inline void timeout_begin(timeout_ctx_t* t)
{
    t->start_tick = HAL_GetTick();
    t->last_tick = t->start_tick;
    t->spin = 0;
}

static inline int timeout_expired(timeout_ctx_t* t, uint32_t timeout_ms)
{
    if (t->spin++ > 1000000UL) {
        return 1; // loop ceiling as last-resort timeout
    }
    uint32_t tick = HAL_GetTick();
    if (tick != t->last_tick) {
        t->last_tick = tick;
    }
    return ((tick - t->start_tick) > timeout_ms);
}

static inline void i2c_clear_errors(I2C_TypeDef* I2Cx)
{
#ifdef I2C_ICR_STOPCF
    I2Cx->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF;
#else
    volatile uint32_t dummy = I2Cx->ISR; (void)dummy;
#endif
}

static inline void i2c_bus_recover(void)
{
    // Only for I2C1 on PB6 (SCL), PB7 (SDA)
    // Save current config
    uint32_t moder = GPIOB->MODER;
    uint32_t otyper = GPIOB->OTYPER;
    uint32_t ospeedr = GPIOB->OSPEEDR;
    uint32_t pupdr = GPIOB->PUPDR;
    uint32_t afr0 = GPIOB->AFR[0];

    // Configure PB6/PB7 as open-drain outputs with pull-ups
    GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER |= (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0); // output
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); // open-drain
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0); // pull-up

    // Release lines
    GPIOB->BSRR = (GPIO_BSRR_BS6 | GPIO_BSRR_BS7);

    // If SDA stuck low, pulse SCL up to 9 times
    for (int i = 0; i < 9; ++i) {
        if ((GPIOB->IDR & GPIO_IDR_ID7) != 0) {
            break; // SDA released
        }
        // Toggle SCL low->high
        GPIOB->BSRR = GPIO_BSRR_BR6;
        for (volatile int d = 0; d < 200; ++d) { __NOP(); }
        GPIOB->BSRR = GPIO_BSRR_BS6;
        for (volatile int d = 0; d < 200; ++d) { __NOP(); }
    }

    // Generate a STOP: SDA low then SCL high then SDA high
    GPIOB->BSRR = GPIO_BSRR_BR7; // SDA low
    for (volatile int d = 0; d < 200; ++d) { __NOP(); }
    GPIOB->BSRR = GPIO_BSRR_BS6; // SCL high
    for (volatile int d = 0; d < 200; ++d) { __NOP(); }
    GPIOB->BSRR = GPIO_BSRR_BS7; // SDA high

    // Restore previous configuration
    GPIOB->AFR[0] = afr0;
    GPIOB->PUPDR = pupdr;
    GPIOB->OSPEEDR = ospeedr;
    GPIOB->OTYPER = otyper;
    GPIOB->MODER = moder;
}

static void i2c_soft_reset_and_reinit(void)
{
#ifdef RCC_APB1RSTR1_I2C1RST
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
#else
    I2C1->CR1 &= ~I2C_CR1_PE;
    for (volatile uint32_t d = 0; d < 1000; ++d) { __NOP(); }
    I2C1->CR1 |= I2C_CR1_PE;
#endif
    // Try to recover bus if a slave is holding SDA low
    i2c_bus_recover();
    TMP275_I2C_Init();
}

static TMP275_Status_t I2C_WriteRegister(TMP275_Handle_t* htmp275, uint8_t reg, uint8_t *data, uint8_t length)
{
    I2C_TypeDef* I2Cx = htmp275->i2c_instance;
    uint8_t device_address = htmp275->device_address << 1;

    // Wait for bus ready with robust timeout and single recovery attempt
    timeout_ctx_t t; timeout_begin(&t);
    while (I2Cx->ISR & I2C_ISR_BUSY) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) {
            i2c_soft_reset_and_reinit();
            timeout_begin(&t);
            while (I2Cx->ISR & I2C_ISR_BUSY) {
                if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
            }
            break;
        }
    }

    i2c_clear_errors(I2Cx);

    I2Cx->CR2 = 0;
    I2Cx->CR2 &= ~I2C_CR2_ADD10;
    I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
    I2Cx->CR2 |= ((length+1) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    I2Cx->CR2 &= ~I2C_CR2_RD_WRN;
    I2Cx->CR2 |= I2C_CR2_START;

    // Wait TXIS
    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
        if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
        if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
    }
    I2Cx->TXDR = reg;

    // Send data
    for (uint16_t i = 0; i < length; i++) {
        timeout_begin(&t);
        while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
            if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
            if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
            if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
        }
        I2Cx->TXDR = data[i];
    }

    // Wait TC
    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_TC)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
        if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
        if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
    }

    // STOP
    I2Cx->CR2 |= I2C_CR2_STOP;
    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_STOPF)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
    }
#ifdef I2C_ICR_STOPCF
    I2Cx->ICR = I2C_ICR_STOPCF;
#endif

    // Wait bus idle
    timeout_begin(&t);
    while (I2Cx->ISR & I2C_ISR_BUSY) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
    }
    return TMP275_OK;
}

static TMP275_Status_t I2C_ReadRegister(TMP275_Handle_t* htmp275, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    I2C_TypeDef* I2Cx = htmp275->i2c_instance;
    uint8_t device_address = htmp275->device_address << 1;

    timeout_ctx_t t; timeout_begin(&t);
    while (I2Cx->ISR & I2C_ISR_BUSY) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) {
            i2c_soft_reset_and_reinit();
            timeout_begin(&t);
            while (I2Cx->ISR & I2C_ISR_BUSY) {
                if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
            }
            break;
        }
    }

    i2c_clear_errors(I2Cx);

    // Write pointer register
    I2Cx->CR2 = 0;
    I2Cx->CR2 &= ~I2C_CR2_ADD10;
    I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
    I2Cx->CR2 |= (1 << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    I2Cx->CR2 &= ~I2C_CR2_RD_WRN;
    I2Cx->CR2 |= I2C_CR2_START;

    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
        if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
        if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
    }
    I2Cx->TXDR = reg;

    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_TC)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
        if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
        if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
    }

    // Read phase
    I2Cx->CR2 = 0;
    I2Cx->CR2 |= device_address & I2C_CR2_SADD_Msk;
    I2Cx->CR2 |= (length << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    I2Cx->CR2 |= I2C_CR2_RD_WRN;
    I2Cx->CR2 |= I2C_CR2_START;

    for (uint16_t i = 0; i < length; i++) {
        timeout_begin(&t);
        while (!(I2Cx->ISR & I2C_ISR_RXNE)) {
            if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
            if (I2Cx->ISR & I2C_ISR_ARLO) { i2c_clear_errors(I2Cx); return TMP275_BUS_ERROR; }
            if (I2Cx->ISR & I2C_ISR_NACKF) { i2c_clear_errors(I2Cx); return TMP275_NACK; }
        }
        buffer[i] = I2Cx->RXDR;
    }

    I2Cx->CR2 |= I2C_CR2_STOP;
    timeout_begin(&t);
    while (!(I2Cx->ISR & I2C_ISR_STOPF)) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
    }
#ifdef I2C_ICR_STOPCF
    I2Cx->ICR = I2C_ICR_STOPCF;
#endif

    timeout_begin(&t);
    while (I2Cx->ISR & I2C_ISR_BUSY) {
        if (timeout_expired(&t, TMP275_I2C_TIMEOUT)) return TMP275_TIMEOUT;
    }

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

    // Clear residual flags before enabling
    i2c_clear_errors(I2C1);

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