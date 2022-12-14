/**
 ******************************************************************************
 * @file    AIC3254.h
 * @author  Carlos Martinez Mora (carmamo.95@gmail.com)
 * @date	 Sep 29, 2022
 * @brief   short description of the file
 *			...
 *
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#ifndef INC_AIC3254_H_
#define INC_AIC3254_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * DEFINES
 */

#define AIC3254_I2C_ADDR	(0x18 << 1)		// 0x30 According to Datasheet, which correspond to 7 bit slave address 0x18

/*
 * REGISTERS
 */

#define PAGE_0				0x00			// Page 0 Register
#define PAGE_1				0x01			// Page 1 Register

/* Page 0 Registers */
#define PAGE_SELECT 		0x00			// Page Select Register
#define SW_RESET			0x01			// Software Reset Register
#define CSR_8				0x12			// Clock Setting Register 8, NADC Values
#define CSR_9				0x13			// Clock Setting Register 9, MADC Values
#define AOSR				0x14			// ADC Oversampling Register
#define PBCR				0x3d			// ADC Signal Processing Block Control Register
#define CH_SETUP			0x51			// ADC Channel Setup Register
#define CH_GAIN				0x52			// ADC Fine Gain Adjust Register

/* Page 1 Registers */
#define POW_CONF			0x01			// Power Configuration Register
#define LDO_CR				0x02			// LDO Control Register
#define CMM_CR				0x0a			// Common Mode Control Register
#define POW_TUN				0x3d			// ADC Power Tune Configuration Register
#define AIQC_CR				0x47			// Analog Input Quick Charging Configuration Register
#define REF_POW_CR			0x7b			// Reference Power-up Configuration Register
#define LROUTE_PCR			0x34			// Left MICPGA Positive Terminal Input Routing Configuration Register
#define LROUTE_NCR			0x36			// Left MICPGA Negative Terminal Input Routing Configuration Register
#define RROUTE_PCR			0x37			// Right MICPGA Positive Terminal Input Routing Configuration Register
#define RROUTE_NCR			0x39			// Right MICPGA Positive Terminal Input Routing Configuration Register
#define LVOLUME_CR			0x3b			// Left MICPGA Volume Control Register
#define RVOLUME_CR			0x3c			// Right MICPGA Volume Control Register


/*
 * CODEC STRUCT
 */

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

} AIC3254_t;

/*
 * INITIALIZE
 */

uint8_t AIC3254_Init(AIC3254_t *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * FILTER SETUP
 */

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef AIC3254_SendCommand(AIC3254_t *dev, uint8_t Register, uint8_t Data);

#ifdef __cplusplus
}
#endif

#endif /* INC_AIC3254_H_ */
