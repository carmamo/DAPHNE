/**
 ******************************************************************************
 * @file    AIC3254.c
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

/*
 * INCLUDES
 */

#include "AIC3254.h"
#include <stdlib.h>

/*
 * INIZIALIZE
 */

uint8_t AIC3254_Init(AIC3254_t *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Init Struct */
	dev->i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;

	/*
	 * Set Stereo ADC with 48ksps Sample Rate and High Performance
	 */

	/* Initialize to Page 0 */

	status = AIC3254_SendCommand(dev, PAGE_SELECT, PAGE_0);
	if(status != HAL_OK) exit(1);

	/* S/W Reset to initialize all registers */

	status = AIC3254_SendCommand(dev, SW_RESET, 0x01);
	if(status != HAL_OK) exit(1);

	/* Power up NADC divider with value 1 */

	status = AIC3254_SendCommand(dev, CSR_8, 0x81);
	if(status != HAL_OK) exit(1);

	/* Power up MADC divider with value 2 */

	status = AIC3254_SendCommand(dev, CSR_9, 0x82);
	if(status != HAL_OK) exit(1);

	/* Program OSR for ADC to 128 */

	status = AIC3254_SendCommand(dev, AOSR, 0x80);
	if(status != HAL_OK) exit(1);

	/* Select ADC PRB_R1 */

	status = AIC3254_SendCommand(dev, PBCR, 0x01);
	if(status != HAL_OK) exit(1);

	/* Select Page 1 */

	status = AIC3254_SendCommand(dev, PAGE_SELECT, PAGE_1);
	if(status != HAL_OK) exit(1);

	/* Disable Internal Crude AVdd in presence of external AVdd supply or before
	 * powering up internal AVdd LDO
	 */

	status = AIC3254_SendCommand(dev, POW_CONF, 0x08);
	if(status != HAL_OK) exit(1);

	/* Enable Master Analog Power Control */

	status = AIC3254_SendCommand(dev, LDO_CR, 0x00);
	if(status != HAL_OK) exit(1);

	/* Set the input common mode to 0.9V */

	status = AIC3254_SendCommand(dev, CMM_CR, 0x00);
	if(status != HAL_OK) exit(1);

	/* Select ADC PTM_R4 */

	status = AIC3254_SendCommand(dev, POW_TUN, 0x00);
	if(status != HAL_OK) exit(1);

	/* Set MicPGA startup delay to 3.1ms */

	status = AIC3254_SendCommand(dev, AIQC_CR, 0x32);
	if(status != HAL_OK) exit(1);

	/* Set the REF charging time to 40ms */

	status = AIC3254_SendCommand(dev, REF_POW_CR, 0x01);
	if(status != HAL_OK) exit(1);

	/* Route IN1L to LEFT_P with 20K input impedance */

	status = AIC3254_SendCommand(dev, LROUTE_PCR, 0x80);
	if(status != HAL_OK) exit(1);

	/* Route Common Mode to LEFT_M with impedance of 20K */

	status = AIC3254_SendCommand(dev, LROUTE_NCR, 0x80);
	if(status != HAL_OK) exit(1);

	/* Route IN1R to Right_P with 20K input impedance */

	status = AIC3254_SendCommand(dev, RROUTE_PCR, 0x80);
	if(status != HAL_OK) exit(1);

	/* Route Common Mode to Right_M with impedance of 20K */

	status = AIC3254_SendCommand(dev, RROUTE_NCR, 0x80);
	if(status != HAL_OK) exit(1);

	/* Unmute Left MICPGA, Gain selection of 6dB to make channel gain 0dB
	 * Register of 6dB with input impedance of 20K -> Channel Gain of 0dB
	 */

	status = AIC3254_SendCommand(dev, LVOLUME_CR, 0x0c);
	if(status != HAL_OK) exit(1);

	/* Unmute Right MICPGA, Gain selection of 6dB to make channel gain 0dB
	 * Register of 6dB with input impedance of 20K -> Channel Gain of 0dB
	 */

	status = AIC3254_SendCommand(dev, RVOLUME_CR, 0x0c);
	if(status != HAL_OK) exit(1);

	/* Select Page 0 */

	status = AIC3254_SendCommand(dev, PAGE_SELECT, PAGE_0);
	if(status != HAL_OK) exit(1);

	/* Power up Left and Right ADC Channels */

	status = AIC3254_SendCommand(dev, CH_SETUP, 0xc0);
	if(status != HAL_OK) exit(1);

	/* Unmute Left and Right ADC Digital Volume Control */

	status = AIC3254_SendCommand(dev, CH_GAIN, 0x00);
	if(status != HAL_OK) exit(1);

	return status;
}


/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef AIC3254_SendCommand(AIC3254_t *dev, uint8_t Register, uint8_t Data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, AIC3254_I2C_ADDR , Register, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);

}
