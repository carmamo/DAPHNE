/**
  ******************************************************************************
  * @file    viterbi.h
  * @author  Carlos Martinez Mora (carmamo.95@gmail.com)
  * @date	 Sep 19, 2022
  * @brief   Implementation of the viterbi algorithm for obtaining the maximum a
  * 		 posteriori probability estimate of the most likely sequence of hidden
  * 		 states (called the Viterbi path) that results in a sequence of observed
  * 		 events, especially in the context of Markov information sources and
  * 		 hidden Markov models (HMM).
  *
  * 		 This algorithm generates a path X = ( x_1 , x_2 , … , x_T ), which is
  * 		 a sequence of states x_n ∈ S = { s_1 , s_2 , … , s_K } that generate
  * 		 the observations Y = ( y_1 , y_2 , … , y_T ) with
  * 		 y_n ∈ O = { o_1 , o_2 , … , o_N }, where N is the number of possible
  * 		 observations in the observation space O.
  *
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
#ifndef INC_VITERBI_H_
#define INC_VITERBI_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DEFINES
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <stm32f4xx_hal.h>

/*
 * VITERBI STRUCT
 */

typedef struct
{
	/* UART Handle */
	UART_HandleTypeDef *UartHandle;

//	/* Transition Matrix */
//	int rows_a;
//	int cols_a;
//	const double** A;
//	/* Emission Matrix */
//	int rows_b;
//	int cols_b;
//	const double** B;
//	/* Sequence of Observations */
//	const double Y[];
//	/* Observation Space */
//
//	/* State Space */
//
//	/* Initial Probabilities */

} VITERBI;

/*
 * FUNCTIONS
 */

uint8_t viterbi_init(VITERBI *god, UART_HandleTypeDef *UartHandle);

int* viterbi_path(const char seq[], int n, int m_t, double A[][m_t], int m_e, double B[][m_e]);

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef VITERBI_send_result(const int path[], int n);

#ifdef __cplusplus
}
#endif

#endif /* INC_VITERBI_H_ */
