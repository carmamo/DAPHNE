/**
 ******************************************************************************
 * @file    lepton.h
 * @author  Carlos Martinez Mora (carmamo.95@gmail.com)
 * @date	 Nov 23, 2022
 * @brief   This file contains the declarations and prototypes for the lepton
 * 			thermal imaging module.
 *			The lepton.h file provides the functionality for initializing and
 *			communicating with the Lepton thermal imaging module.
 *			It includes functions for setting and getting register values,
 *			sending and receiving data through I2C, and enabling/disabling
 *			vsync and radiometry.
 *			This header file is an interface between the application and the
 *			Lepton module, providing an easy-to-use way for accessing the
 *			module's features.
 *
 ******************************************************************************
 * @attention
 * Please make sure to read the Lepton datasheet for further information on the
 * communication protocol and the module's features.
 * The code is provided "as is" and the author is not responsible for any errors
 * The or damages that may occur from its use.
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
/**
 * @def PACKET_SIZE_U8
 * @brief Total size of a single frame in bytes.
 *
 */
#define PACKET_SIZE__2_5_U8		(164)

#define PACKET_SIZE__3_5_U8		(324)
/**
 * @def FRAME_HEIGHT
 * @brief Height of a single frame in pixels.
 *
 */
#define FRAME_HEIGHT_2_5	(60)

#define FRAME_HEIGHT_3_5	(120)
/**
 * @def FRAME_WIDTH
 * @brief Width of a single frame in pixels.
 *
 */
#define FRAME_WIDTH_2_5		(80)

#define FRAME_WIDTH_3_5		(160)
/**
 * @def LEPTON_I2C_ADDR
 * @brief The I2C address of the Lepton module.
 *
 */
#define LEPTON_I2C_ADDR	(0x2A << 1)

/**
 * @struct lepton_t
 * @brief This struct holds handles for I2C, SPI, CS GPIO and UART communication interfaces.
 *
 * @var lepton_t::i2cHandle
 * Pointer to I2C handle.
 * @var lepton_t::spiHandle
 * Pointer to SPI handle.
 * @var lepton_t::CS_port
 * Pointer to GPIO port used for chip select.
 * @var lepton_t::CS_pin
 * Pin number used for chip select.
 * @var lepton_t::uartHandle
 * Pointer to UART handle.
 */
typedef struct {

	/* I2C handle */
	I2C_HandleTypeDef	*i2cHandle;
	/* SPI handle */
	SPI_HandleTypeDef	*spiHandle;
	/* CS GPIO Handle */
	GPIO_TypeDef 		*CS_port;
	uint16_t			CS_pin;
	/* UART handle */
	UART_HandleTypeDef	*uartHandle;

} lepton_t;

/**
 * @struct vospi_packet
 * @brief This struct holds two header values and an array of data.
 * @var vospi_packet::header
 * An array of two 16-bit header values.
 * @var vospi_packet::data
 * An array of 16-bit data values.
 */
typedef struct {

	uint16_t header[2];
	uint16_t data[FRAME_WIDTH];

} vospi_packet;

/**
 * @struct lepton_frame
 * @brief This struct holds an array of vospi_packet structures, representing a frame of data.
 * @var lepton_frame::y16
 * An array of vospi_packet structures.
 */
typedef struct {

	vospi_packet y16[FRAME_HEIGHT];

} lepton_frame;

/**
 * @fn void lepton_Init(I2C_HandleTypeDef*, SPI_HandleTypeDef*, UART_HandleTypeDef*, GPIO_TypeDef*, uint16_t)
 * @brief Initializes the Lepton module with the given handles and GPIO information
 *
 * @param i2c Handle for the I2C communication
 * @param spi Handle for the SPI communication
 * @param uart Handle for the UART communication
 * @param port GPIO port for the chip select signal
 * @param pin GPIO pin for the chip select signal
 */
void lepton_Init(I2C_HandleTypeDef *i2c, SPI_HandleTypeDef *spi, UART_HandleTypeDef *uart, GPIO_TypeDef *port, uint16_t pin);

/**
 * @fn HAL_StatusTypeDef lepton_SetReg(uint8_t)
 * @brief Sets a register in the Lepton module
 *
 * @param reg The register to set
 * @return The result of the register set operation
 *
 * in depth description of the function
 */
HAL_StatusTypeDef lepton_SetReg(uint8_t reg);

/**
 * @fn HAL_StatusTypeDef lepton_GetReg(uint16_t, uint16_t*)
 * @brief Gets the value of a register in the Lepton module
 *
 * @param reg The register to get
 * @param rxdata The value of the register
 * @return The result of the register get operation
 */
HAL_StatusTypeDef lepton_GetReg(uint16_t reg, uint16_t *rxdata);

/**
 * @fn HAL_StatusTypeDef lepton_SetData(uint8_t*, uint16_t)
 * @brief Sets data in the Lepton module
 *
 * @param txdata The data to set
 * @param size The size of the data
 * @return The result of the data set operation
 */
HAL_StatusTypeDef lepton_SetData(uint8_t *txdata, uint16_t size);

/**
 * @fn HAL_StatusTypeDef lepton_GetData(uint16_t*, uint16_t)
 * @brief Gets data from the Lepton module
 *
 * @param rxdata The data received
 * @param size The size of the data
 * @return The result of the data get operation
 */
HAL_StatusTypeDef lepton_GetData(uint16_t *rxdata, uint16_t size);

/**
 * @fn HAL_StatusTypeDef lepton_command(uint16_t)
 * @brief Sends a command to the Lepton module
 *
 * @param cmd The command to send
 * @return The result of the command operation
 */
HAL_StatusTypeDef lepton_command(uint16_t cmd);

/**
 * @fn HAL_StatusTypeDef lepton_radiometry(bool)
 * @brief Sets the radiometry mode on the Lepton module
 *
 * @param rad_enabled Boolean indicating whether radiometry mode should be enabled
 * @return The result of the radiometry mode set operation
 */
HAL_StatusTypeDef lepton_radiometry(bool rad_enabled);

/**
 * @fn HAL_StatusTypeDef lepton_vsync(bool)
 * @brief Sets the vsync mode on the Lepton module
 *
 * @param vsync_enabled Boolean indicating whether vsync mode should be enabled
 * @return The result of the vsync mode set operation
 */
HAL_StatusTypeDef lepton_vsync(bool vsync_enabled);

/**
 * @fn HAL_StatusTypeDef lepton_version()
 * @brief
 *
 * @return
 */
HAL_StatusTypeDef lepton_version();

/**
 * @fn void send_byte(uint8_t)
 * @brief Sends a single byte of data over the specified communication interface.
 *
 * @param data The data to be sent, as a single byte.
 */
void send_byte(uint8_t data);

/**
 * @fn void send_msg(int)
 * @brief Function to send a message over the UART interface.
 * This function sends the integer data in string format over the UART interface
 * with a line feed and carriage return appended at the end. The UART transmission
 * is done using the HAL_UART_Transmit function with a timeout of 100ms.
 *
 * @param data Integer data to be transmitted over UART.
 */
void send_msg(int data);

#ifdef __cplusplus
}
#endif

#endif /* INC_LEPTON_H_ */
