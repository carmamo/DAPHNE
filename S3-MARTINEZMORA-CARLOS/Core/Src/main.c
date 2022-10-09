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
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for button */
osThreadId_t buttonHandle;
const osThreadAttr_t button_attributes = {
  .name = "button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Stepper */
osThreadId_t StepperHandle;
const osThreadAttr_t Stepper_attributes = {
  .name = "Stepper",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for led */
osThreadId_t ledHandle;
const osThreadAttr_t led_attributes = {
  .name = "led",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Serial */
osThreadId_t SerialHandle;
const osThreadAttr_t Serial_attributes = {
  .name = "Serial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/***************************************************************************/
/* Pines Bobinas STEPPER */
/***************************************************************************/
uint16_t pines[4] = {GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_10};
/***************************************************************************/
/* Matriz Estados STEPPER */
/***************************************************************************/
GPIO_PinState matrix[5][4] = {
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET},
		{GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET},
		{GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET},
		{GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET}};
/***************************************************************************/
/* Estados del STEPPER */
/***************************************************************************/
enum giro {
	Antihorario, Horario
};

enum giro giro_stepper;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void check_status(void *argument);
void stepper_run(void *argument);
void blink(void *argument);
void serial_receive(void *argument);

/* USER CODE BEGIN PFP */
static int debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
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
  /* creation of button */
  buttonHandle = osThreadNew(check_status, NULL, &button_attributes);

  /* creation of Stepper */
  StepperHandle = osThreadNew(stepper_run, NULL, &Stepper_attributes);

  /* creation of led */
  ledHandle = osThreadNew(blink, NULL, &led_attributes);

  /* creation of Serial */
  SerialHandle = osThreadNew(serial_receive, NULL, &Serial_attributes);

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
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static int debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	static uint16_t rebote = 0;

	rebote = (rebote << 1) | HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) | 0xfe00;
	return (rebote <= 0xfff0);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_check_status */
/**
  * @brief  Function implementing the button thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_check_status */
void check_status(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	  if(debounce(GPIOC, GPIO_PIN_13))
	  {
		  while(debounce(GPIOC, GPIO_PIN_13))
		  {
			  osThreadSuspend(StepperHandle);
			  for(int i = 0; i < 4; i++)
			  {
				  HAL_GPIO_WritePin(GPIOB, pines[i], matrix[4][i]);
			  }
		  }
		  if (giro_stepper == Horario)
		  {
			  giro_stepper = Antihorario;
		  }
		  else
		  {
			  giro_stepper = Horario;

		  }
	  }

	  switch(giro_stepper)
	  {
//	  case Parado1:
//	  {
//		  osThreadSuspend(StepperHandle);
//		  for(int i = 0; i < 4; i++)
//		  {
//			  HAL_GPIO_WritePin(GPIOB, pines[i], matrix[4][i]);
//		  }
//		  break;
//	  }
//	  case Parado2:
//	  {
//		  osThreadSuspend(StepperHandle);
//		  for(int i = 0; i < 4; i++)
//		  {
//			  HAL_GPIO_WritePin(GPIOB, pines[i], matrix[4][i]);
//		  }
//		  break;
//	  }
	  case Horario:
	  {
		  osThreadResume(StepperHandle);
		  break;
	  }
	  case Antihorario:
	  {
		  osThreadResume(StepperHandle);
		  break;
	  }
	  default:{
		  break;
	  }
	  }


    osDelay(20);
  }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_stepper_run */
/**
* @brief Function implementing the stepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stepper_run */
void stepper_run(void *argument)
{
  /* USER CODE BEGIN stepper_run */
	int ciclo = 0;

  /* Infinite loop */
  for(;;)
  {
	  switch(giro_stepper)
	  {
	  case Horario:
	  {
		  for(int i = 0; i < 4; i++)
		  {
			  HAL_GPIO_WritePin(GPIOB, pines[i], matrix[ciclo][i]);
		  }
		  ciclo++;
		  if(ciclo > 3) ciclo = 0;
		  break;
	  }
	  case Antihorario:
	  {
		  for(int i = 0; i < 4; i++)
		  {
			  HAL_GPIO_WritePin(GPIOB, pines[i], matrix[ciclo][i]);
		  }
		  ciclo--;
		  if(ciclo < 0) ciclo = 3;
		  break;
	  }
	  default:
	  {
		  break;
	  }
	  }




    osDelay(2);
  }
  /* USER CODE END stepper_run */
}

/* USER CODE BEGIN Header_blink */
/**
* @brief Function implementing the led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink */
void blink(void *argument)
{
	/* USER CODE BEGIN blink */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		osDelay(250);
	}
	/* USER CODE END blink */
}

/* USER CODE BEGIN Header_serial_receive */
/**
* @brief Function implementing the Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serial_receive */
void serial_receive(void *argument)
{
  /* USER CODE BEGIN serial_receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END serial_receive */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
