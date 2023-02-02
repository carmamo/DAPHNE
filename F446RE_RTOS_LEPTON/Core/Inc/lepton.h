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
#include <stdbool.h>

/*
 * DEFINES
 */

#define FRAME_SIZE		(164)
#define FRAME_HEIGHT	(60)
#define FRAME_WIDTH		(80)

#define LEPTON_I2C_ADDR	(0x2A << 1)

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

typedef struct {

	uint16_t header[2];
	uint16_t data[FRAME_WIDTH];

} vospi_packet;

typedef struct {

	vospi_packet y16[FRAME_HEIGHT];

} lepton_frame;

void lepton_Init(I2C_HandleTypeDef *i2c, SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin);

void print_image_binary_background(void);

void transfer(void);

void lepton_getPacket(void);

HAL_StatusTypeDef lepton_SetReg(uint8_t reg);

HAL_StatusTypeDef lepton_GetReg(uint16_t reg, uint16_t *rxdata);

HAL_StatusTypeDef lepton_SetData(uint8_t *txdata, uint16_t size);

HAL_StatusTypeDef lepton_GetData(uint16_t *rxdata, uint16_t size);

HAL_StatusTypeDef lepton_command(uint16_t cmd);

HAL_StatusTypeDef lepton_radiometry(bool rad_enabled);

HAL_StatusTypeDef lepton_vsync(bool vsync_enabled);


void send_byte(uint8_t data);

void send_msg(int data);

#ifdef __cplusplus
}
#endif

#endif /* INC_LEPTON_H_ */
