/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "CANSPI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN1 &hspi1
#define CAN2 &hspi2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_sem */
osSemaphoreId_t CAN_semHandle;
const osSemaphoreAttr_t CAN_sem_attributes = {
  .name = "CAN_sem"
};
/* Definitions for uart_tx_sem */
osSemaphoreId_t uart_tx_semHandle;
const osSemaphoreAttr_t uart_tx_sem_attributes = {
  .name = "uart_tx_sem"
};
/* USER CODE BEGIN PV */
uint8_t rx_data;
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
bool enable_CAN = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void can_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t tx_buffer[21];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CAN_sem */
  CAN_semHandle = osSemaphoreNew(1, 1, &CAN_sem_attributes);

  /* creation of uart_tx_sem */
  uart_tx_semHandle = osSemaphoreNew(1, 1, &uart_tx_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN */
  CANHandle = osThreadNew(can_task, NULL, &CAN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Ican1_Pin || GPIO_Pin==Ican2_Pin) {
		osSemaphoreRelease(CAN_semHandle); //, &priority);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	osSemaphoreRelease(uart_tx_semHandle); //, &priority);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_can_task */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_task */
void can_task(void *argument)
{
  /* USER CODE BEGIN can_task */
	uint8_t tx_buffer[49], i=0;

	if( CANSPI_Initialize(CAN1)) {
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "CAN1 initialized OK\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 15;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = 1;
		txMessage.frame.data1 = 2;
		txMessage.frame.data2 = 3;
		txMessage.frame.data3 = 4;
		txMessage.frame.data4 = 5;
		txMessage.frame.data5 = 6;
		txMessage.frame.data6 = 7;
		txMessage.frame.data7 = 8;
		CANSPI_Transmit(CAN1, &txMessage);
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "CAN1 sending Msg...\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	}
	else
	{
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "Error CAN1 initiali\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	}
	if (CANSPI_Initialize(CAN2)) {
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "CAN2 initialized OK\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 25;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = 8;
		txMessage.frame.data1 = 7;
		txMessage.frame.data2 = 6;
		txMessage.frame.data3 = 5;
		txMessage.frame.data4 = 4;
		txMessage.frame.data5 = 3;
		txMessage.frame.data6 = 2;
		txMessage.frame.data7 = 1;
		CANSPI_Transmit(CAN2, &txMessage);
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "CAN2 sending Msg...\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	}
	else
	{
		osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		sprintf(tx_buffer, "Error CAN2 initiali\n\r");
		HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));

	}
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(CAN_semHandle, portMAX_DELAY);
		if(CANSPI_Receive(CAN1, &rxMessage))
		{
			// Coger el semáforo de envío de trama por la UART
			osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
			sprintf(tx_buffer, "CAN1 RX: id= %2d, dlc= %1d, datos= %1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d\n\r",
					rxMessage.frame.id, rxMessage.frame.dlc,rxMessage.frame.data0,rxMessage.frame.data1,
					rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,
					rxMessage.frame.data6,rxMessage.frame.data7);
			HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));

			txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
			txMessage.frame.id = 15;
			txMessage.frame.dlc = 8;
			txMessage.frame.data0 = 1;
			txMessage.frame.data1 = 2;
			txMessage.frame.data2 = 3;
			txMessage.frame.data3 = 4;
			txMessage.frame.data4 = 5;
			txMessage.frame.data5 = 6;
			txMessage.frame.data6 = 7;
			txMessage.frame.data7 = 8;
			CANSPI_Transmit(CAN1, &txMessage);
			osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
			for (i=0; i<49; i++) tx_buffer[i]=0;
			sprintf(tx_buffer, "CAN1 sending Msg...\n\r");
			HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		} else
			if(CANSPI_Receive(CAN2, &rxMessage))
			{
				// Coger el semáforo de envío de trama por la UART
				osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
				sprintf(tx_buffer, "CAN2 RX: id= %2d, dlc= %1d, datos= %1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d\n\r",
						rxMessage.frame.id, rxMessage.frame.dlc,rxMessage.frame.data0,rxMessage.frame.data1,
						rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,
						rxMessage.frame.data6,rxMessage.frame.data7);
				HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
				CANSPI_CL_Flag_Int(CAN2);
				txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
				txMessage.frame.id = 25;
				txMessage.frame.dlc = 8;
				txMessage.frame.data0 = 8;
				txMessage.frame.data1 = 7;
				txMessage.frame.data2 = 6;
				txMessage.frame.data3 = 5;
				txMessage.frame.data4 = 4;
				txMessage.frame.data5 = 3;
				txMessage.frame.data6 = 2;
				txMessage.frame.data7 = 1;
				CANSPI_Transmit(CAN2, &txMessage);
				osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
				for (i=0; i<49; i++) tx_buffer[i]=0;
				sprintf(tx_buffer, "CAN2 sending Msg...\n\r");
				HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
			}
		osDelay(1000);
	}
  /* USER CODE END can_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
