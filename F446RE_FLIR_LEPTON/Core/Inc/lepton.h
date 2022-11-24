/**
 ******************************************************************************
 * @file    lepton.h
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
#ifndef INC_LEPTON_H_
#define INC_LEPTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * DEFINES
 */

#define FRAME_SIZE	(164)

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef	*i2cHandle;
	/* SPI handle */
	SPI_HandleTypeDef	*spiHandle;
	/* CS GPIO Hanlde */
	GPIO_TypeDef 		*CS_port;
	uint16_t			CS_pin;
	/* UART handle */
	UART_HandleTypeDef	*uartHandle;



} lepton_t;

void lepton_Init(SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin);

void print_image_binary_background(void);

void transfer(void);

void send_byte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* INC_LEPTON_H_ */
