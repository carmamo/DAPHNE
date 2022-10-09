/**
  ******************************************************************************
  * @file    viterbi.c
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

#include "viterbi.h"

/*
 * LOW-LEVEL FUNCTIONS
 */

static double max(double a, double b)
{
    return a > b ? a : b;
}

static int argmax(double a, double b)
{
    return a > b ? 0 : 1;
}

HAL_StatusTypeDef VITERBI_send_result(const int path[], int n)
{

	return 0;
}

/*
 * FUNCTIONS
 */

uint8_t viterbi_init(VITERBI *god, UART_HandleTypeDef *UartHandle)
{
	god->UartHandle = UartHandle;

	return 0;
}

int* viterbi_path(const char seq[], int n, int m_t, double A[][m_t], int m_e, double B[][m_e])
{
	int *path = 0;
	double vprob[1] = vprob[0] + n;
	double* ptr[2] = { vprob[1] + n, vprob[1] + 2 * n };
	double* pi[2] = { ptr[1] + n, ptr[1] + 2 * n };

	// initialize vprob array; assumed starting state is state F
	vprob[0][0] = 1;
	vprob[1][0] = 0;

	// viterbi algorithm in log space to avoid underflow
	for (int i = 1; i < n; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			double row0 = (vprob[0][i - 1] + A[0][j]);
			double row1 = (vprob[1][i - 1] + A[1][j]);

			vprob[j][i] = B[seq[i]][j] + max( row0, row1 );
			ptr[j][i] = argmax( row0, row1 );
			pi[j][i] = max( row0 , row1 );
		}
	}

	// traceback to find most likely path
	path[n - 1] = argmax( pi[0][n - 1], pi[1][n - 1] );
	for (int i = n - 2; i > 0; i--)
	{
		path[i] = ptr[path[i + 1]][i + 1];
	}
	return path;
}


