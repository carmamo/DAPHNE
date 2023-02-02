/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _WaveHeader{
	char riff[4];
	uint32_t size;
	char wave[4];
	char fmt[4];
	uint32_t fmt_size;
	uint16_t format; //1:PCM
	uint16_t channels; // channels
	uint32_t sampleRate;  // sample rate
	uint32_t rbc;//sampleRate*bitsPerSample*channels/8
	uint16_t bc; //bitsPerSample*channels/8
	uint16_t bitsPerSample; //bitsPerSample
	char data[4];
	uint32_t data_size;
} WAVE_HEADER;

typedef enum {
    STATE_IDLE,
	CONNECTING,
    STATE_START_RECORDING,
    STATE_RECORDING,
    STATE_STOP
} AUDIO_STATE;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_COUNT	(40960)

#define I2S_DATA_WORD_LENGTH	(24)						// industry-standard 24-bit I2S
#define I2S_FRAME				(32)						// bits per sample
#define READ_SIZE				(256)						// samples to read from I2S
#define BUFFER_SIZE				(READ_SIZE*I2S_FRAME/16)	// number of uint16_t elements expected
#define WRITE_SIZE_BYTES		(BUFFER_SIZE*2)				// bytes to write

#define I2S_SAMPLE_FREQUENCY	(16000)						// sample frequency

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2S */
osThreadId_t I2SHandle;
const osThreadAttr_t I2S_attributes = {
  .name = "I2S",
  .stack_size = 246 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WAV */
osThreadId_t WAVHandle;
const osThreadAttr_t WAV_attributes = {
  .name = "WAV",
  .stack_size = 1920 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RxCommandSem */
osSemaphoreId_t RxCommandSemHandle;
const osSemaphoreAttr_t RxCommandSem_attributes = {
  .name = "RxCommandSem"
};
/* Definitions for RxAudioSem */
osSemaphoreId_t RxAudioSemHandle;
const osSemaphoreAttr_t RxAudioSem_attributes = {
  .name = "RxAudioSem"
};
/* USER CODE BEGIN PV */
static AUDIO_STATE audio_state = CONNECTING;
FRESULT res;
FIL file_ptr;

uint16_t aud_buf[2*BUFFER_SIZE];			// Double buffering
static volatile int16_t *BufPtr;

static char buf[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void *argument);
void pvrCommandReceiveTask(void *argument);
void pvrWriteAudioTask(void *argument);
void pvrWriteWavFileTask(void *argument);

/* USER CODE BEGIN PFP */
FRESULT fwrite_wav_header(FIL* file, uint16_t sampleRate, uint8_t bitsPerSample, uint8_t channels);
FRESULT Format_SD (void);
void convert_endianness(uint32_t *array, uint16_t Size);
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
  MX_I2S2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RxCommandSem */
  RxCommandSemHandle = osSemaphoreNew(1, 1, &RxCommandSem_attributes);

  /* creation of RxAudioSem */
  RxAudioSemHandle = osSemaphoreNew(1, 1, &RxAudioSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  RxCommandSemHandle = osSemaphoreNew(1, 0, &RxCommandSem_attributes);

  RxAudioSemHandle = osSemaphoreNew(1, 0, &RxAudioSem_attributes);

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

  /* creation of UART */
  UARTHandle = osThreadNew(pvrCommandReceiveTask, NULL, &UART_attributes);

  /* creation of I2S */
  I2SHandle = osThreadNew(pvrWriteAudioTask, NULL, &I2S_attributes);

  /* creation of WAV */
  WAVHandle = osThreadNew(pvrWriteWavFileTask, NULL, &WAV_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CODEC_Reset_GPIO_Port, CODEC_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_Reset_Pin */
  GPIO_InitStruct.Pin = CODEC_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CODEC_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BSP_API_SDIO_Pin */
  GPIO_InitStruct.Pin = BSP_API_SDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_API_SDIO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	osSemaphoreRelease(RxCommandSemHandle);
	HAL_UARTEx_ReceiveToIdle_IT(huart, (uint8_t *)buf, 4);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	BufPtr = aud_buf;
	osSemaphoreRelease(RxAudioSemHandle);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	BufPtr = &aud_buf[BUFFER_SIZE];
	osSemaphoreRelease(RxAudioSemHandle);
}

FRESULT fwrite_wav_header(FIL* file, uint16_t sampleRate, uint8_t bitsPerSample, uint8_t channels) {
	static UINT *bw;
	WAVE_HEADER wave_header;
	wave_header.riff[0] = 'R';wave_header.riff[1] = 'I';
	wave_header.riff[2] = 'F';wave_header.riff[3] = 'F';
	wave_header.size = (uint32_t)0;
	wave_header.wave[0] = 'W';wave_header.wave[1] = 'A';
	wave_header.wave[2] = 'V';wave_header.wave[3] = 'E';
	wave_header.fmt[0] = 'f';wave_header.fmt[1] = 'm';
	wave_header.fmt[2] = 't';wave_header.fmt[3] = ' ';
	wave_header.fmt_size = 16;
	wave_header.format = 1; 								// PCM
	wave_header.channels = channels; 						// channels
	wave_header.sampleRate=sampleRate;  					// sample rate
	wave_header.rbc = sampleRate*bitsPerSample*channels/8;
	wave_header.bc =  bitsPerSample*channels/8;
	wave_header.bitsPerSample = bitsPerSample; 				// bitsPerSample
	wave_header.data[0] = 'd'; wave_header.data[1] = 'a';
	wave_header.data[2] = 't'; wave_header.data[3] = 'a';
	wave_header.data_size = 0;
	return f_write(file, (uint8_t*)&wave_header, sizeof(wave_header), bw);
}

FRESULT Format_SD (void)
{
	DIR dir;
	static FILINFO fno;
	static FRESULT fresult;

	char *path = malloc(20*sizeof (char));
	sprintf (path, "%s","/");

	fresult = f_opendir(&dir, path);                       /* Open the directory */
	if (fresult == FR_OK)
	{
		for (;;)
		{
			fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR)     /* It is a directory */
			{
				if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
				fresult = f_unlink(fno.fname);
				if (fresult == FR_DENIED) continue;
			}
			else
			{   /* It is a file. */
				fresult = f_unlink(fno.fname);
			}
		}
		f_closedir(&dir);
	}
	free(path);
	return fresult;
}

/*
This function takes in a pointer to an array of 32-bit unsigned integers
and a 16-bit unsigned integer (Size) as input.
It converts the endianness of each element in the array using the __REV
intrinsic function.
The for loop iterates through each element in the array, and at each iteration,
the endianness of the current element is converted.
*/
void convert_endianness(uint32_t *array, uint16_t Size) {
    for (int i = 0; i < Size; i++) {
        array[i] = __REV(array[i]);
    }
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
    osThreadSuspend(defaultTaskHandle);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pvrCommandReceiveTask */
/**
* @brief
* This function is a task that is responsible for receiving commands from the UART
* interface.
* It first initializes the UART receive interrupt using the HAL_UARTEx_ReceiveToIdle_IT()
* function.
* The task enters an infinite loop, where it waits for the RxCommandSemHandle semaphore
* to be acquired.
* Once the semaphore is acquired, the task checks the first byte of the received buffer
* (buf) to determine which command was received.
*
* 	If the first byte is 'G', it checks if the audio state is in the IDLE state, and if
* 	so, it resumes the WAVHandle thread.
* 	If the first byte is 'P', it checks if the audio state is in the RECORDING state, and
* 	if so, it sets the audio state to STOP.
* 	If the first byte is '.', it sets the audio state to CONNECTING.
* 	If the first byte is '+', it sets the audio state to IDLE.
* 	If the first byte is any other value, the task does nothing.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pvrCommandReceiveTask */
void pvrCommandReceiveTask(void *argument)
{
  /* USER CODE BEGIN pvrCommandReceiveTask */
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)buf, 4);
  /* Infinite loop */
	for(;;)
	{

		// Wait for semaphore to be released, data has been received on the UART.
		osSemaphoreAcquire(RxCommandSemHandle, osWaitForever);

		// Check the first byte of the received data to determine the command.
		switch(buf[0])
		{
		case 'G':
			// If the command is 'G', and the audio state is currently idle,
			// resume the WAV recording task.
			if(audio_state == STATE_IDLE) osThreadResume(WAVHandle);
			break;
		case 'P':
			// If the command is 'P', and the audio state is currently recording,
			// change the state to stop recording.
			while(audio_state == STATE_START_RECORDING) osDelay(500);
			if(audio_state == STATE_RECORDING) audio_state = STATE_STOP;
			break;
		case '.':
			audio_state = CONNECTING;
			break;
		case '+':
			audio_state = STATE_IDLE;
			break;
		default:
			// If the command is not recognized, do nothing.
			break;
		}
	}
  /* USER CODE END pvrCommandReceiveTask */
}

/* USER CODE BEGIN Header_pvrWriteAudioTask */
/**
* @brief Function implementing the I2S thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pvrWriteAudioTask */
void pvrWriteAudioTask(void *argument)
{
  /* USER CODE BEGIN pvrWriteAudioTask */
	static UINT *bw;

	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(RxAudioSemHandle, osWaitForever);

		convert_endianness((uint32_t *)BufPtr, READ_SIZE);
		res = f_write(&file_ptr, BufPtr, WRITE_SIZE_BYTES, bw);
	}
  /* USER CODE END pvrWriteAudioTask */
}

/* USER CODE BEGIN Header_pvrWriteWavFileTask */
/**
* @brief Function implementing the SDIO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pvrWriteWavFileTask */
void pvrWriteWavFileTask(void *argument)
{
  /* USER CODE BEGIN pvrWriteWavFileTask */

	// Initialize variables
	static uint16_t count = 0;	 	// counter for creating unique filenames
	uint32_t filesize;				// variable to store the size of the recorded file
	uint32_t data_len;				// variable to store the length of the data in the file
	uint32_t total_len;				// variable to store the total length of the file (including header)
	static char filename[256];		// variable to store the filename of the recorded file
	static UINT *bw;				// variable for fwrite function

	// Mount the SD card and format it if necessary
	do
	{
		res = f_mount(&SDFatFS, SDPath, 1);
	}
	while( res != FR_OK);

	do
	{
		res = Format_SD();
	}
	while (res != FR_OK);
	/* Infinite loop */
	for(;;)
	{
		switch(audio_state)
		{
		case STATE_START_RECORDING:

			// Create a unique filename for the recorded file
			sprintf(filename, "%saud_%03d.wav", SDPath, count++);

			// Open the file in write mode
			do
			{
				res = f_open(&file_ptr, filename, FA_CREATE_ALWAYS|FA_WRITE);
			}
			while(res != FR_OK);

			// Write the WAV header to the file
			res = fwrite_wav_header(&file_ptr, I2S_SAMPLE_FREQUENCY, I2S_FRAME, 2);

			// Start recording audio using DMA
			HAL_I2S_Receive_DMA(&hi2s2, aud_buf, 2*READ_SIZE);
			audio_state = STATE_RECORDING;
			break;

		case STATE_RECORDING:

			// Wait for 50ms before checking the state again
			osDelay(50);
			break;

		case STATE_STOP:

			// Stop the DMA recording
			HAL_I2S_DMAStop(&hi2s2);

			// Wait for any remaining audio data to be processed
			while(osSemaphoreGetCount(RxAudioSemHandle)) osDelay(250);

			// Get the size of the file and calculate the length of the data and total length
			filesize = f_size(&file_ptr);
			data_len = filesize - 44;
			total_len = filesize - 8;

			// Update the WAV header with the correct data length and total length
			f_lseek(&file_ptr, 4);
			f_write(&file_ptr, (uint8_t*)&total_len, 4, bw);
			f_lseek(&file_ptr, 40);
			f_write(&file_ptr, (uint8_t*)&data_len, 4, bw);

			// Close the file
			f_close(&file_ptr);


			// Set the audio state to idle
			audio_state = STATE_IDLE;
			break;

		case STATE_IDLE:

			// Suspend the task until the next recording is started
			osThreadSuspend(WAVHandle);
			audio_state = STATE_START_RECORDING;
			break;

		default:
			osDelay(50);
			break;
		}
	}
  /* USER CODE END pvrWriteWavFileTask */
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