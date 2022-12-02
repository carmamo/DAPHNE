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
#include "u8g2.h"
#include "ICI2022.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SERVO */
osThreadId_t SERVOHandle;
const osThreadAttr_t SERVO_attributes = {
  .name = "SERVO",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for STEPPER */
osThreadId_t STEPPERHandle;
const osThreadAttr_t STEPPER_attributes = {
  .name = "STEPPER",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SERIAL */
osThreadId_t SERIALHandle;
const osThreadAttr_t SERIAL_attributes = {
  .name = "SERIAL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LOCK */
osThreadId_t LOCKHandle;
const osThreadAttr_t LOCK_attributes = {
  .name = "LOCK",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for lock_queue */
osMessageQueueId_t lock_queueHandle;
const osMessageQueueAttr_t lock_queue_attributes = {
  .name = "lock_queue"
};
/* Definitions for Adc_Sem */
osSemaphoreId_t Adc_SemHandle;
const osSemaphoreAttr_t Adc_Sem_attributes = {
  .name = "Adc_Sem"
};
/* Definitions for Rx_Sem */
osSemaphoreId_t Rx_SemHandle;
const osSemaphoreAttr_t Rx_Sem_attributes = {
  .name = "Rx_Sem"
};
/* USER CODE BEGIN PV */
static adc_q adc_value = {0, 0, 0.0, 0.0, true};
static lock_q segvel_value;
static char buf[5];
static unsigned short periodo = 2;
static enum giro giro_stepper;
u8g2_t u8g2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void oled_update(void *argument);
void adc_read(void *argument);
void servo_pos(void *argument);
void stepper_fx(void *argument);
void serial_fx(void *argument);
void servo_lock(void *argument);

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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ICI2022_Init(&hi2c1, &huart2);
  calc_rpm();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Adc_Sem */
  Adc_SemHandle = osSemaphoreNew(1, 1, &Adc_Sem_attributes);

  /* creation of Rx_Sem */
  Rx_SemHandle = osSemaphoreNew(1, 1, &Rx_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  Rx_SemHandle = osSemaphoreNew(1, 0, &Rx_Sem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of lock_queue */
  lock_queueHandle = osMessageQueueNew (5, sizeof(lock_q), &lock_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(oled_update, NULL, &OLED_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(adc_read, NULL, &ADC_attributes);

  /* creation of SERVO */
  SERVOHandle = osThreadNew(servo_pos, NULL, &SERVO_attributes);

  /* creation of STEPPER */
  STEPPERHandle = osThreadNew(stepper_fx, NULL, &STEPPER_attributes);

  /* creation of SERIAL */
  SERIALHandle = osThreadNew(serial_fx, NULL, &SERIAL_attributes);

  /* creation of LOCK */
  LOCKHandle = osThreadNew(servo_lock, NULL, &LOCK_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(adc_value.first_conv)
	{
		adc_value.adc_ch0 = HAL_ADC_GetValue(hadc);
		adc_value.temp = (adc_value.adc_ch0/4096.0) * 3.3;
		adc_value.temp = adc_value.temp/0.01;
		HAL_ADC_Start_IT(hadc);

	}
	else
	{
		adc_value.adc_ch1 = HAL_ADC_GetValue(hadc);
		adc_value.pot = (adc_value.adc_ch1/4096.0) * 3.3;
		osSemaphoreRelease(Adc_SemHandle);
		HAL_ADC_Stop_IT(hadc);
	}
	adc_value.first_conv = !adc_value.first_conv;

}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	osSemaphoreRelease(Rx_SemHandle);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)buf, 5);
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
    osDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_oled_update */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oled_update */
void oled_update(void *argument)
{
  /* USER CODE BEGIN oled_update */
	static int needle_offset_x_left;
	static int needle_offset_y_left;
	static int needle_offset_x_right;
	static int needle_offset_y_right;
	static int right_offset = 68;
	static char temp[5];
	static char pos[5];
	static char speed[10];
	static needle left = {
			.small = 10,
			.big = 25,
			.center_x = 29,
			.center_y = 31,
			.start_x = 0,
			.start_y = 0,
			.end_x = 0,
			.end_y = 0,
			.angle = 45 };
	static needle right = {
			.small = 10,
			.big = 25,
			.center_x = 29 + 68,
			.center_y = 31,
			.start_x = 0,
			.start_y = 0,
			.end_x = 0,
			.end_y = 0,
			.angle = 45 };

	u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	u8g2_SetI2CAddress(&u8g2, OLED_ADDR);	// 0x78
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
  /* Infinite loop */
	for(;;)
	{
		left.angle = map(__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2), 99, 199, 45, 270+45);
		right.angle = map(rpm[periodo-2], 0, 15, 45, 270+45);


		calc_needle(&left);
		calc_needle(&right);

		sprintf(temp, "%0.2f", adc_value.temp);
		sprintf(pos, "%li", map(__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2), 99, 199, 0, 90));
		sprintf(speed, "%0.2fRPM", rpm[periodo-2]);

		if((left.angle > 45 && left.angle < 135) || (left.angle > 225 && left.angle < 315))
		{
			needle_offset_x_left = 0;
			needle_offset_y_left = 1;
		}
		else
		{
			needle_offset_x_left = 1;
			needle_offset_y_left = 0;
		}
		if((right.angle > 45 && right.angle < 135) || (right.angle > 225 && right.angle < 315))
		{
			needle_offset_x_right = 0;
			needle_offset_y_right = 1;
		}
		else
		{
			needle_offset_x_right = 1;
			needle_offset_y_right = 0;
		}

		u8g2_FirstPage(&u8g2);
		do {

			// draw gauges & termometer
			u8g2_DrawBitmap(&u8g2, 0, 0, 64/8, 56, Left_Gauge);
			u8g2_DrawBitmap(&u8g2, 64, 0, 64/8, 56, Right_Gauge);
			u8g2_DrawBitmap(&u8g2, 60, 0, 8/8, 56, Termometer);

			// draw temperature
			u8g2_DrawLine(&u8g2, 64, 49, 64, 39-map(adc_value.adc_ch0, 0, 4096, 0, 49));
			u8g2_DrawStr(&u8g2, 52, 64, temp);

			// draw needle and center circle Left Gauge
			u8g2_DrawLine(&u8g2, left.start_x, left.start_y, left.end_x, left.end_y);
			u8g2_DrawLine(&u8g2, left.start_x + needle_offset_x_left, left.start_y + needle_offset_y_left,
								 left.end_x + needle_offset_x_left, left.end_y + needle_offset_y_left);
			u8g2_DrawStr(&u8g2, 26, 64, pos);


			u8g2_DrawBitmap(&u8g2, 26, 28, 8/8, 8, Contour_Needle);

			// draw needle and center circle Right Gauge
			u8g2_DrawLine(&u8g2, right.start_x, right.start_y, right.end_x, right.end_y);
			u8g2_DrawLine(&u8g2, right.start_x + needle_offset_x_right, right.start_y + needle_offset_y_right,
								 right.end_x + needle_offset_x_right, right.end_y + needle_offset_y_right);
			u8g2_DrawStr(&u8g2, 26 + right_offset, 64, speed);


			u8g2_DrawBitmap(&u8g2, 26 + right_offset, 28, 8/8, 8, Contour_Needle);

		} while (u8g2_NextPage(&u8g2));

		osDelay(250);
	}
  /* USER CODE END oled_update */
}

/* USER CODE BEGIN Header_adc_read */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adc_read */
void adc_read(void *argument)
{
  /* USER CODE BEGIN adc_read */
	HAL_ADCEx_Calibration_Start(&hadc);
	static uint8_t trama[4] = {0x30, 0x00, 0x00, 0xE0};
	static uint32_t timestamp;
	timestamp = HAL_GetTick();
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(Adc_SemHandle, osWaitForever);
		HAL_ADC_Start_IT(&hadc);

		if(HAL_GetTick() - timestamp >= 1000)
		{
			trama[1] = (uint8_t)adc_value.temp;
			trama[2] = (uint8_t)((adc_value.temp - trama[1])*100);
			HAL_UART_Transmit(&huart2, trama, 4, 100);
			timestamp = HAL_GetTick();
		}


		osDelay(10);
	}
  /* USER CODE END adc_read */
}

/* USER CODE BEGIN Header_servo_pos */
/**
* @brief Function implementing the SERVO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_pos */
void servo_pos(void *argument)
{
  /* USER CODE BEGIN servo_pos */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	/* Infinite loop */
	for(;;)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, map(adc_value.adc_ch1, 0, 4096, 99, 199));
		osDelay(10);

	}
  /* USER CODE END servo_pos */
}

/* USER CODE BEGIN Header_stepper_fx */
/**
* @brief Function implementing the STEPPER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stepper_fx */
void stepper_fx(void *argument)
{
  /* USER CODE BEGIN stepper_fx */
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
			HAL_GPIO_WritePin(GPIOB, pins[i], matrix_half[ciclo][i]);		// Medios pasos, quitar _half para paso completo
		}
		ciclo++;
		//			if(ciclo > 3) ciclo = 0;											// Pasos completos
		if(ciclo > 7) ciclo = 0;											// Medios pasos
		break;
	}
	case Antihorario:
	{
		for(int i = 0; i < 4; i++)
		{
			HAL_GPIO_WritePin(GPIOB, pins[i], matrix_half[ciclo][i]);		// Medios pasos, quitar _half para paso completo
		}
		ciclo--;
		//			if(ciclo < 0) ciclo = 3;											// Pasos completos
		if(ciclo < 0) ciclo = 7;											// Medios pasos
		break;
	}
	default:
	{
		break;
	}
	}
	osDelay(periodo);
}
  /* USER CODE END stepper_fx */
}

/* USER CODE BEGIN Header_serial_fx */
/**
* @brief Function implementing the SERIAL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serial_fx */
void serial_fx(void *argument)
{
  /* USER CODE BEGIN serial_fx */
	static uint16_t angulo = 0;
	static char msg[50];

	osThreadSuspend(OLEDHandle);
	osThreadSuspend(LOCKHandle);
	u8g2_SetPowerSave(&u8g2, 1); // sleep display
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)buf, 5);
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(Rx_SemHandle, osWaitForever);
		switch(buf[0])
		{
		case 'P':
		{
			send_uart("'P': Servo controlado por potenciometro...\r\n\n");
			osThreadResume(SERVOHandle);
			osThreadSuspend(LOCKHandle);
			break;
		}
		case 'S':
		{
			osThreadSuspend(SERVOHandle);
			osThreadSuspend(LOCKHandle);

			send_uart("'S': Introduce el angulo deseado (0-90):\r\n\n");
			osSemaphoreAcquire(Rx_SemHandle, osWaitForever);
			angulo = atol(buf);
			sprintf(msg,"Angulo recibido: %d\r\n\n", angulo);
			send_uart(msg);
			angulo = map(angulo, 0, 90, 99, 199);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, angulo);
			break;
		}
		case 'A':
		{
			osThreadSuspend(SERVOHandle);
			osThreadResume(LOCKHandle);

//			send_uart("'A': Introduce la espera deseada: (0-255):\r\n\n");
//			osSemaphoreAcquire(Rx_SemHandle, osWaitForever);
//			segvel_value.espera = atol(buf);
//			sprintf(msg,"Espera recibida: %d segundos\r\n\n", segvel_value.espera);
//			send_uart(msg);
//			send_uart("'A': Introduce la velocidad deseada: (0-255):\r\n\n");
//			osSemaphoreAcquire(Rx_SemHandle, osWaitForever);
//			segvel_value.velocidad = atol(buf);
//			if(segvel_value.velocidad == 0) segvel_value.velocidad++;
//			sprintf(msg,"Velocidad recibida: %d grados/segundo\r\n\n", segvel_value.velocidad);
//			send_uart(msg);
			segvel_value.espera = segvel_value.espera * 1000;
			segvel_value.velocidad = 1000/segvel_value.velocidad;
			osMessageQueuePut(lock_queueHandle, &segvel_value, 1, 0);
			break;
		}
		case 'D':
		{
			send_uart("'D': Sentido Horario...\r\n\n");
			giro_stepper = Horario;
			break;
		}
		case 'I':
		{
			send_uart("'I': Sentido Antihorario...\r\n\n");
			giro_stepper = Antihorario;
			break;
		}
		case 'M':
		{
			send_uart("'M': Introduzca el PERIODO (1-9):\r\n\n");
			osSemaphoreAcquire(Rx_SemHandle, osWaitForever);
			if(atol(buf) <= 1)
			{
				periodo = 2;
				send_uart("'M': Recibido 1...\r\n\n");
				break;
			}
			if(atol(buf) >= 9)
			{
				periodo = 10;
				send_uart("'M': Recibido 9...\r\n\n");
				break;
			}
			periodo = atol(buf) + 1;
			sprintf(msg, "'M': Recibido %hu...\r\n\n", periodo - 1);
			send_uart(msg);
			break;
		}
		case 'X':
			osThreadResume(OLEDHandle);
			u8g2_SetPowerSave(&u8g2, 0); // wake up display
			break;
		case 'C':
			osThreadSuspend(OLEDHandle);
			u8g2_SetPowerSave(&u8g2, 1); // sleep display
			break;
		case 'T':
			if(buf[3] != 0xE0) break;

		default:
		{
			send_uart("ERROR!! Comando no valido\r\n\n");
		}
		}
		osDelay(1);

	}
  /* USER CODE END serial_fx */
}

/* USER CODE BEGIN Header_servo_lock */
/**
* @brief Function implementing the LOCK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_lock */
void servo_lock(void *argument)
{
  /* USER CODE BEGIN servo_lock */
	enum maquina {
		Reposo, Abriendo, Espera, Cerrando
	} estados;
	estados = Reposo;
	/* Infinite loop */
	for(;;)
	{
		switch(estados)
		{
		case Reposo:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 99);
			osMessageQueueGet(lock_queueHandle, &segvel_value, (uint8_t *)1, osWaitForever);
			estados = Abriendo;
			break;
		case Abriendo:
			for(int i = 0; i<91; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, map(i, 0, 90, 99, 199));
				osDelay(segvel_value.velocidad);
			}
			estados = Espera;
			break;
		case Espera:
			osDelay(segvel_value.espera);
			estados = Cerrando;
			break;
		case Cerrando:
			for(int i = 90; i>=0; i--)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, map(i, 0, 90, 99, 199));
				osDelay(segvel_value.velocidad);
			}
			estados = Reposo;
			if(osMessageQueueGetCount(lock_queueHandle) == 0) osThreadSuspend(LOCKHandle);
			break;
		default:
			break;

		}
		osDelay(1);
	}
  /* USER CODE END servo_lock */
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
