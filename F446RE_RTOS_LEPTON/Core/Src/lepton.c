/**
 ******************************************************************************
 * @file    lepton.c
 * @author  Carlos Martinez Mora (carmamo.95@gmail.com)
 * @date	 Nov 23, 2022
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

#include "lepton.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/**
  @defgroup Lepton Lepton Driver

 */

/**
  @addtogroup Lepton
  @{
 */


static lepton_t dev;


void lepton_Init(I2C_HandleTypeDef *i2c, SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin)
{
	dev.spiHandle = spi;
	dev.uartHandle = uart;
	dev.i2cHandle = i2c;
	dev.CS_port = port;
	dev.CS_pin = pin;
}
//HAL_GPIO_WritePin(FLIR_PWR_DWN_L_GPIO_Port, FLIR_PWR_DWN_L_Pin, GPIO_PIN_RESET);
//HAL_Delay(100);
//HAL_GPIO_WritePin(FLIR_PWR_DWN_L_GPIO_Port, FLIR_PWR_DWN_L_Pin, GPIO_PIN_SET);
//HAL_GPIO_WritePin(FLIR_RESET_L_GPIO_Port, FLIR_RESET_L_Pin, GPIO_PIN_RESET);
//HAL_Delay(100);
//HAL_GPIO_WritePin(FLIR_RESET_L_GPIO_Port, FLIR_RESET_L_Pin, GPIO_PIN_SET);
//HAL_Delay(5000);


HAL_StatusTypeDef lepton_SetReg(uint8_t reg)
{
	static uint8_t array[2] = {0x00, 0x00};

	array[1] = reg;
	return HAL_I2C_Master_Transmit(dev.i2cHandle, LEPTON_I2C_ADDR, array, 2, 100);
}

HAL_StatusTypeDef lepton_GetReg(uint16_t reg, uint16_t *rxdata)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(dev.i2cHandle, LEPTON_I2C_ADDR, reg, 2, (uint8_t *)rxdata, 2, 100);
	*rxdata = (*rxdata >> 8 | *rxdata << 8);
	return status;

//	lepton_SetReg(reg);
//	return HAL_I2C_Master_Receive(dev.i2cHandle, LEPTON_I2C_ADDR, (uint8_t *)rxdata, 2, 100);

}
HAL_StatusTypeDef lepton_SetData(uint8_t *txdata, uint16_t size)
{
	HAL_StatusTypeDef status;
	uint16_t *rxdata = (uint16_t *)malloc(sizeof(uint16_t));

	do
	{
		lepton_GetReg(0x0002, rxdata);
	}
	while(*rxdata & 0x01);

//	HAL_I2C_Master_Transmit(dev.i2cHandle, LEPTON_I2C_ADDR, txdata, size, 100);

	// Write data, 0x0008 Data 0 Address (beginning of data block)
	status = HAL_I2C_Mem_Write(dev.i2cHandle, LEPTON_I2C_ADDR, 0x0008, 2, txdata, size, 100);
	if(status != HAL_OK)
	{
		free(rxdata);
		return status;
	}

	// Write data length, 0x0006 Data Length Register
	size = size >> 1;
	size = (size >> 8 | size << 8);
	status = HAL_I2C_Mem_Write(dev.i2cHandle, LEPTON_I2C_ADDR, 0x0006, 2, (uint8_t *)&size, 2, 100);

	free(rxdata);
	return status;
}

HAL_StatusTypeDef lepton_GetData(uint16_t *rxdata, uint16_t size)
{
	HAL_StatusTypeDef status;

	do
	{
		lepton_GetReg(0x0002, rxdata);
	}
	while(*rxdata & 0x01);

	status = HAL_I2C_Mem_Read(dev.i2cHandle, LEPTON_I2C_ADDR, 0x0006, 2, (uint8_t *)rxdata, 2, 100);
	if( status != HAL_OK) return status;
	if(*rxdata < size) return HAL_ERROR;

	status = HAL_I2C_Master_Receive(dev.i2cHandle, LEPTON_I2C_ADDR, (uint8_t *)rxdata, size, 100);
//	*rxdata = (*rxdata >> 8 | *rxdata << 8);
	return status;
}

HAL_StatusTypeDef lepton_command(uint16_t cmd)
{
	uint16_t *rxdata = (uint16_t *)malloc(sizeof(uint16_t));

	do
	{
		lepton_GetReg(0x02, rxdata);
	}
	while(*rxdata & 0x01);

	free(rxdata);
	cmd = (cmd >> 8 | cmd << 8);
	return HAL_I2C_Mem_Write(dev.i2cHandle, LEPTON_I2C_ADDR, 0x0004, 2, (uint8_t *)&cmd, 2, 100);
}

HAL_StatusTypeDef lepton_radiometry(bool rad_enabled)
{
	uint8_t *data = (uint8_t *)malloc(4*sizeof(uint8_t));
	for(int i=0; i<4; i++)
	{
		data[i] = 0x00;
	}

	if(rad_enabled) data[1] = 0x01;

	lepton_SetData(data, 4);
	free(data);

	/* 0x0E00 (RAD Module ID) + 0x10 (Command Base - Ctrl enable) + 0x01 (Set) + 0x4000 (Protection Bit) = 0x4E11 */
	return lepton_command(0x4E11);
}

HAL_StatusTypeDef lepton_vsync(bool vsync_enabled)
{
	uint8_t *data = (uint8_t *)malloc(4*sizeof(uint8_t));
		for(int i=0; i<4; i++)
		{
			data[i] = 0x00;
		}

		if(vsync_enabled) data[1] = 0x05;

		lepton_SetData(data, 4);
		free(data);

		/* 0x0800 (OEM Module ID) + 0x54 (Command Base - Ctrl enable) + 0x01 (Set) + 0x4000 (Protection Bit) = 0x4855 */
		return lepton_command(0x4855);
}

HAL_StatusTypeDef lepton_version()
{
	if(lepton_command(0x481C) != HAL_OK) return HAL_ERROR;

	char leptonhw[33];
	lepton_GetData((uint16_t *)leptonhw, 32);

	//Detected Lepton2.5 Shuttered (Radiometric)
	if (strstr(leptonhw, "05-070360") != NULL)
	{
		return HAL_OK;
	}

	//Detected Lepton3.5 Shuttered (Radiometric)
	else if (strstr(leptonhw, "05-070170") != NULL)
	{
		return HAL_OK;
	}

	//Unsupported Lepton
	else
	{
		return HAL_ERROR;
	}



}


void send_byte(uint8_t data)
{
	HAL_UART_Transmit(dev.uartHandle, &data, 1, 100);
}
void send_msg(int data)
{
	static char buf[5];
	sprintf(buf, "%d\r\n", data);
	HAL_UART_Transmit(dev.uartHandle, (uint8_t *)buf, strlen(buf), 100);
}

/**
  @} end of Lepton group
 */
