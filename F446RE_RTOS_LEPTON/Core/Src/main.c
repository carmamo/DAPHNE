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
/*! \mainpage RTOS LEPTON Software Library
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 * \section install_sec Using the Library
 *
 * \subsection step1 Step 1: Opening the box
 *
 * etc...
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lepton.h"
#include "arm_math.h"
#include <stdbool.h>
#include <string.h>
#include "SEGGER_SYSVIEW.h"
#include "portable.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/**
  @defgroup Main Main Program

 */

/**
  @addtogroup Main
  @{
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEMPOOL_OBJECTS (4)
#define FRAME_SIZE_U8	(9840)
#define FRAME_SIZE_3_5_U8	(9840)
#define MEMPOOL_SIZE	(39360)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* Definitions for uartTx */
osThreadId_t uartTxHandle;
const osThreadAttr_t uartTx_attributes = {
  .name = "uartTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VoSPI */
osThreadId_t VoSPIHandle;
const osThreadAttr_t VoSPI_attributes = {
  .name = "VoSPI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

#endif /* DOXYGEN_SHOULD_SKIP_THIS */

lepton_frame MemoryBlock[MEMPOOL_OBJECTS]; ///< Static memory allocation for memory pool

static uint8_t lost_frame = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Definitions for FramePool */

osMemoryPoolId_t FramePoolHandle;
const osMemoryPoolAttr_t FramePool_attributes = {
  .name = "FramePool",
  .mp_mem = MemoryBlock,
  .mp_size = MEMPOOL_SIZE
};

lepton_frame *current_frame;
lepton_frame *complete_frame;

static uint16_t frame_packet[(FRAME_HEIGHT*FRAME_WIDTH) + 2] = {0xDEAD, 0xBEEF};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void prvPrintImageTask(void *argument);
void prvCaptureFramesTask(void *argument);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();

  SEGGER_SYSVIEW_Conf();					/* Configure and initialize SystemView */
  vSetVarulMaxPRIGROUPValue();
  SEGGER_SYSVIEW_Start();

  lepton_Init(&hi2c1, &hspi1, &huart2, SPI1_CS_GPIO_Port, SPI1_CS_Pin);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  FramePoolHandle = osMemoryPoolNew(MEMPOOL_OBJECTS, FRAME_SIZE_U8, &FramePool_attributes);
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of uartTx */
  uartTxHandle = osThreadNew(prvPrintImageTask, NULL, &uartTx_attributes);

  /* creation of VoSPI */
  VoSPIHandle = osThreadNew(prvCaptureFramesTask, NULL, &VoSPI_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLIR_PWR_DWN_L_Pin|FLIR_RESET_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLIR_VSYNC_Pin */
  GPIO_InitStruct.Pin = FLIR_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLIR_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLIR_PWR_DWN_L_Pin FLIR_RESET_L_Pin */
  GPIO_InitStruct.Pin = FLIR_PWR_DWN_L_Pin|FLIR_RESET_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/**
 * @fn void HAL_GPIO_EXTI_Callback(uint16_t)
 * @brief
 * This function is a callback routine called when an external
 * interrupt is generated by a specified GPIO pin.
 *
 * @param GPIO_Pin Specifies the pin number for the interrupt line.
 * The function checks if the interrupt was generated by the specified
 * FLIR_VSYNC_Pin and if so, allocates memory from the FramePoolHandle
 * memory pool. If memory allocation is successful, it starts a DMA
 * transfer to receive data from the hspi1 peripheral and disables the
 * EXTI15_10_IRQn interrupt.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SEGGER_SYSVIEW_RecordEnterISR();
	if (GPIO_Pin == FLIR_VSYNC_Pin)
	{
		current_frame = (lepton_frame *)osMemoryPoolAlloc(FramePoolHandle, 0U);
		if(current_frame != NULL)
		{
			osThreadFlagsSet(VoSPIHandle, 0x1U);
			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		}
	}
	SEGGER_SYSVIEW_RecordExitISR();
}
/**
 * @fn void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef*)
 * @brief
 *
 * @param hspi
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SEGGER_SYSVIEW_RecordEnterISR();
	osThreadFlagsSet(VoSPIHandle, 0x2U);
	SEGGER_SYSVIEW_RecordExitISR();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_prvPrintImageTask */
/**
* @brief Function implementing the uart_tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvPrintImageTask */
void prvPrintImageTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for(;;)
	{
		/* Wait for completed frame */
		osThreadFlagsWait(0x1U, osFlagsWaitAny, osWaitForever);

		for(int i = 0; i < 60; i++)
		{
			memcpy((void *)&frame_packet[2 + (i*80)], (void *)complete_frame->y16[i].data, 160);
		}

		osMemoryPoolFree(FramePoolHandle, complete_frame);
		complete_frame = NULL;


		/* Transmit frame */
		if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
		{
			CDC_Transmit_FS((uint8_t *)frame_packet, (FRAME_SIZE_U8 + 4));
		}


		__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_prvCaptureFramesTask */
/**
* @brief Function implementing the VoSPI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvCaptureFramesTask */
void prvCaptureFramesTask(void *argument)
{
  /* USER CODE BEGIN prvCaptureFramesTask */


	HAL_Delay(100);
	HAL_GPIO_WritePin(FLIR_PWR_DWN_L_GPIO_Port, FLIR_PWR_DWN_L_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FLIR_RESET_L_GPIO_Port, FLIR_RESET_L_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);

	lepton_vsync(true);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	/* Infinite loop */
	for(;;)
	{
		osThreadFlagsWait(0x1U, osFlagsWaitAny, osWaitForever);

		HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)current_frame, FRAME_SIZE_U8);

		osThreadFlagsWait(0x2U, osFlagsWaitAny, osWaitForever);


		if((current_frame->y16[59].header[0] & 0xff) == (FRAME_HEIGHT - 1))
		{
			complete_frame = current_frame;
			current_frame = NULL;

			lost_frame = 0;

			osThreadFlagsSet(uartTxHandle, 0x1U);

		}
		else
		{
			lost_frame++;

			if(lost_frame > 9)
			{
				// Synchronization Lost
				osDelay(185);

//				do
//				{
//					HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)current_frame, 1);
//					osThreadFlagsWait(0x2U, osFlagsWaitAny, osWaitForever);
//
//				} while((current_frame->y16[0].header[0] & 0xf00) == 0xf00);

				HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)current_frame , FRAME_SIZE_U8);

				osThreadFlagsWait(0x2U, osFlagsWaitAny, osWaitForever);

				complete_frame = current_frame;
				current_frame = NULL;

				lost_frame = 0;

				osThreadFlagsSet(uartTxHandle, 0x1U);
			}
			else
			{
				osMemoryPoolFree(FramePoolHandle, current_frame);
				current_frame = NULL;

				__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			}
		}
	}
  /* USER CODE END prvCaptureFramesTask */
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
