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

uint8_t lepton_frame_packet[FRAME_SIZE];
int lepton_image[80][80];

int image_state = -1;
int print_image_binary_i;
int print_image_binary_j;

int lost_frame_counter = 0;
int last_frame_number;
int frame_complete = 0;
int start_image = 0;
int last_crc;
int new_frame = 0;
int frame_counter = 0;
int frame_number;
uint8_t tx[1] = {0x00};

static lepton_t dev;


void lepton_Init(I2C_HandleTypeDef *i2c, SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin)
{
	dev.spiHandle = spi;
	dev.uartHandle = uart;
	dev.i2cHandle = i2c;
	dev.CS_port = port;
	dev.CS_pin = pin;
}

void print_image_binary_background(void)
{
	switch(image_state)
	{
	case 0:
		send_byte(0xDE);
		image_state++;
		break;
	case 1:
		send_byte(0xAD);
		image_state++;
		break;
	case 2:
		send_byte(0xBE);
		image_state++;
		break;
	case 3:
		send_byte(0xEF);
		image_state++;
		print_image_binary_i = 0;
		print_image_binary_j = 0;
		break;
	case 4:
		send_byte((lepton_image[print_image_binary_i][print_image_binary_j] >> 8)&0xff);
		send_byte((lepton_image[print_image_binary_i][print_image_binary_j])&0xff);

		print_image_binary_j++;
		if(print_image_binary_j >= 80)
		{
			print_image_binary_j=0;
			print_image_binary_i++;
			if(print_image_binary_i >= 60) image_state = -1;
		}
		break;
	default:
		break;
	}

}

void transfer(void)
{
	HAL_GPIO_WritePin(dev.CS_port, dev.CS_pin, GPIO_PIN_RESET);
//	HAL_SPI_Receive_IT(dev.spiHandle, lepton_frame_packet, FRAME_SIZE);
	HAL_SPI_Receive(dev.spiHandle, lepton_frame_packet, FRAME_SIZE,1000);
	HAL_GPIO_WritePin(dev.CS_port, dev.CS_pin, GPIO_PIN_SET);

	if((lepton_frame_packet[0] & 0xf) != 0x0f)
	{
		if(lepton_frame_packet[1] == 0)
		{
			if(last_crc != (lepton_frame_packet[3] << 8 | lepton_frame_packet[4]))
			{
				lost_frame_counter = 0;
				new_frame = 1;
			}
			last_crc = lepton_frame_packet[3] << 8 | lepton_frame_packet[4];

		}

		frame_number = lepton_frame_packet[1];

		if(frame_number < 60)
		{

			if(image_state == -1)
			{
				for(int i = 0; i < 80; i++)
				{
					lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);
				}
			}
		}
		else
		{
			lost_frame_counter++;
		}



		if(frame_number == 59)
		{
			frame_complete = 1;
//			last_frame_number = 0;
		}
	}

	lost_frame_counter++;
	if(lost_frame_counter > 100)
	{
		HAL_Delay(185);				// RESYNC
		lost_frame_counter = 0;
	}


	if(frame_complete)
	{
		if(new_frame)
		{
			frame_counter++;
			image_state = 0;
			new_frame = 0;
		}
		frame_complete = 0;
	}
}

void lepton_getPacket(void)
{
	HAL_GPIO_WritePin(dev.CS_port, dev.CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Receive_IT(dev.spiHandle, lepton_frame_packet, FRAME_SIZE);
}

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
