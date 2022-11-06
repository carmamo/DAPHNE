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

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
// Fondo, 128x64px
const unsigned char background[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00,
	0x00, 0x00, 0x1e, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x78, 0x00, 0x00,
	0x00, 0x00, 0x70, 0x10, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x10, 0x0e, 0x00, 0x00,
	0x00, 0x03, 0x80, 0x10, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x10, 0x01, 0xc0, 0x00,
	0x00, 0x06, 0x00, 0x10, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x60, 0x00,
	0x00, 0x08, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x10, 0x00,
	0x00, 0x30, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x0c, 0x00,
	0x00, 0x60, 0x01, 0x2f, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x06, 0x00,
	0x00, 0x80, 0x01, 0x28, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x20, 0x01, 0xee, 0x00, 0x04, 0x80, 0x00, 0x00, 0x01, 0x20, 0x00, 0x00, 0x00, 0x04, 0x80,
	0x02, 0x10, 0x00, 0x21, 0x00, 0x08, 0x40, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x08, 0x40,
	0x06, 0x08, 0x00, 0x21, 0x00, 0x10, 0x60, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x00, 0x10, 0x60,
	0x04, 0x04, 0x00, 0x2e, 0x00, 0x20, 0x20, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x20, 0x20,
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
	0x18, 0x73, 0xc0, 0x00, 0x01, 0xde, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
	0x10, 0x0a, 0x40, 0x00, 0x02, 0x12, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x10, 0x72, 0x40, 0x00, 0x03, 0x92, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x20, 0x0a, 0x40, 0x00, 0x02, 0x52, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x20, 0x0a, 0x40, 0x00, 0x02, 0x52, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x60, 0x73, 0xc0, 0x00, 0x01, 0x9e, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x81, 0x78, 0x00, 0x00, 0x00, 0xf7, 0x81, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x81, 0x40, 0x00, 0x00, 0x00, 0x14, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x81, 0x70, 0x00, 0x00, 0x00, 0x17, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0xbd, 0x08, 0x00, 0x00, 0x00, 0x20, 0xbd, 0x00, 0x00, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d,
	0x81, 0x08, 0x00, 0x00, 0x00, 0x40, 0x81, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x81, 0x70, 0x00, 0x00, 0x00, 0x87, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x10, 0x01, 0xe0, 0x00, 0xf7, 0x80, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x18, 0x01, 0x20, 0x00, 0x94, 0x80, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
	0x08, 0x01, 0x20, 0x00, 0xf4, 0x80, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
	0x04, 0x05, 0x20, 0x00, 0x14, 0xa0, 0x20, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x20, 0x20,
	0x04, 0x09, 0x20, 0x00, 0x14, 0x90, 0x20, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00, 0x00, 0x10, 0x20,
	0x02, 0x11, 0xe0, 0x00, 0xf7, 0x88, 0x40, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x08, 0x40,
	0x01, 0x20, 0x00, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x01, 0x20, 0x00, 0x00, 0x00, 0x04, 0x80,
	0x00, 0xc0, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void oled_update(void *argument);

/* USER CODE BEGIN PFP */
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(oled_update, NULL, &OLED_attributes);

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
  hadc.Init.DiscontinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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

}

/* USER CODE BEGIN 4 */
// Byte GPIO and Delay Callback
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		osDelay(1);
		break;
	case U8X8_MSG_DELAY_MILLI:
		osDelay(arg_int);
		break;
	default:
		u8x8_SetGPIOResult(u8x8, 1);			// default return value
		break;
	}
	return 1;
}

// Communication Callback
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg)
	{
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while( arg_int > 0 )
		{
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		/* add your custom code to init i2c subsystem */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		/* ignored for i2c */
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8) << 1, buffer, buf_idx, 1000);
		break;
	default:
		return 0;
	}
	return 1;
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
	u8g2_t u8g2;

	u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_stm32_gpio_and_delay);
	u8g2_SetI2CAddress(&u8g2,0x3C);	// 0x78 = 0x3C << 1
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
  /* Infinite loop */
	for(;;)
	{
		u8g2_FirstPage(&u8g2);
		do {

			// dibujar background y barra medidora
			u8g2_DrawBitmap(&u8g2, 0, 0, 128/8, 64, background);

		} while (u8g2_NextPage(&u8g2));
		osDelay(1);
	}
  /* USER CODE END oled_update */
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
