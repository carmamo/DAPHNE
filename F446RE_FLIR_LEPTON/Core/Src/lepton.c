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


void lepton_Init(SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin)
{
	dev.spiHandle = spi;
	dev.uartHandle = uart;
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
	for(int i = 0; i < FRAME_SIZE; i++)
	{
		HAL_SPI_TransmitReceive(dev.spiHandle,&tx[0], &lepton_frame_packet[i], 1, 100);
	}
	HAL_GPIO_WritePin(dev.CS_port, dev.CS_pin, GPIO_PIN_SET);

	send_byte(lepton_frame_packet[0]);
	if((lepton_frame_packet[0] & 0xf) != 0x0f)
	{
		if(lepton_frame_packet[1] == 0)
		{
			if(last_crc != (lepton_frame_packet[3] << 8 | lepton_frame_packet[4]))
			{
				new_frame = 1;
			}
			last_crc = lepton_frame_packet[3] << 8 | lepton_frame_packet[4];
		}

		frame_number = lepton_frame_packet[1];

		if(frame_number < 60)
		{
			lost_frame_counter = 0;
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

		send_byte(frame_number);

		if(frame_number == 59)
		{
			frame_complete = 1;
			last_frame_number = 0;
		}
	}

	lost_frame_counter++;
	if(lost_frame_counter > 100)
	{
		HAL_Delay(185);				// RESYNC
		lost_frame_counter = 0;
	}


	if(frame_complete && new_frame)
	{

		frame_counter++;
		if(frame_counter%18 == 0)
		{
			image_state = 0;
		}
		new_frame = 0;
		frame_complete = 0;
	}
}

void send_byte(uint8_t data)
{
	HAL_UART_Transmit(dev.uartHandle, &data, 1, 100);
}
