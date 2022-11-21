/**
 ******************************************************************************
 * @file    ICI2022.h
 * @author  Carlos Martinez Mora (carmamo.95@gmail.com)
 * @date	 Nov 10, 2022
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
#ifndef INC_ICI2022_H_
#define INC_ICI2022_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "u8g2.h"
#include <stdbool.h>

/*
 * DEFINES
 */

#define OLED_ADDR	0x78 // OLED slave address in 7 bit = 0x3C

/*
 * VARIABLES
 */

// ' Termometer', 8x56px
extern const unsigned char Termometer [];
// ' Contour_Needle', 8x8px
extern const unsigned char Contour_Needle [];
// ' Fill_Cntr_Needle', 8x8px
extern const unsigned char Fill_Cntr_Needle [];
// ' Right_Gauge', 64x56px
extern const unsigned char Right_Gauge [];
// ' Left_Gauge', 64x56px
extern const unsigned char Left_Gauge [];

/***************************************************************************/
/* Velocidad en RPM */
/***************************************************************************/
extern float steps_sec;
extern float rpm[9];
/***************************************************************************/
/* Pines Bobinas STEPPER */
/***************************************************************************/
extern const uint16_t pins[4];
/***************************************************************************/
/* Matriz Estados STEPPER */
/***************************************************************************/
extern const GPIO_PinState matrix[5][4];
/***************************************************************************/
/* Matriz Estados Medios Pasos STEPPER */
/***************************************************************************/
extern const GPIO_PinState matrix_half[9][4];
/***************************************************************************/
/* Estados del STEPPER */
/***************************************************************************/
enum giro {
	Antihorario, Horario, Parado
};

/*
 * TYPEDEF STRUCTS
 */

typedef struct
{
	uint16_t adc_ch0;
	uint16_t adc_ch1;
	float temp;
	float pot;
	bool first_conv;
} adc_q;
typedef struct
{
	uint16_t espera;
	uint16_t velocidad;
} lock_q;
typedef struct
{
	int small;		// Length of small line in px
	int big;		// Length of big line	in px
	int center_x;	// Center x position in px
	int center_y;	// Center y position in px
	int start_x;
	int start_y;
	int end_x;
	int end_y;
	int angle;
} needle;

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	UART_HandleTypeDef *uartHandle;
	ADC_HandleTypeDef *adcHandle;
	TIM_HandleTypeDef *timHandle;

} ICI2022_t;

/*
 * FUNCTION PROTOTYPES
 */

/*
 * INITIALIZE
 */

void ICI2022_Init(I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uartHandle);

/*
 * LOW-LEVEL OLED FUNCTIONS
 */
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

/*
 * LOW-LEVEL STEPPER FUNCTIONS
 */

void stepper_Init(char *step_mode);

void step(void);

/*
 * MATH FUNCTIONS
 */

uint32_t map(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax);

double radians(double degrees);

void calc_needle(needle *s);

void calc_rpm(void);

/*
 * LOW-LEVEL FUNCTIONS
 */

void send_uart(char *string);



#ifdef __cplusplus
}
#endif

#endif /* INC_ICI2022_H_ */
