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
#include "sdcard.h"
#include "ff.h"
#include "rtc_stm32f1.h"

#include "math.h"
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
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for write_SDcard */
osThreadId_t write_SDcardHandle;
const osThreadAttr_t write_SDcard_attributes = {
  .name = "write_SDcard",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2s_task */
osThreadId_t i2s_taskHandle;
const osThreadAttr_t i2s_task_attributes = {
  .name = "i2s_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2s_Queue */
osMessageQueueId_t i2s_QueueHandle;
const osMessageQueueAttr_t i2s_Queue_attributes = {
  .name = "i2s_Queue"
};
/* Definitions for i2s_ready */
osSemaphoreId_t i2s_readyHandle;
const osSemaphoreAttr_t i2s_ready_attributes = {
  .name = "i2s_ready"
};
/* USER CODE BEGIN PV */
FATFS fs;
FRESULT res;

char root[256] = {};

int16_t i2s_buf[1024];
int16_t i2s_buf_s[512];
//int16_t i2s_buf2[512];
uint8_t i2s_ready = 0;

FIL wav_file;
uint32_t wav_file_num_samples = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2S3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */
static void MX_SPI2_InitFast(void);
void open_wav(void);
void close_wav(void);
void update_wav(uint16_t *buff, uint16_t size);

void i2s_rx_complete(I2S_HandleTypeDef *hi2s);
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
  MX_SPI2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of i2s_ready */
  i2s_readyHandle = osSemaphoreNew(1, 0, &i2s_ready_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of i2s_Queue */
  i2s_QueueHandle = osMessageQueueNew (4, sizeof(uint16_t *), &i2s_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of write_SDcard */
  write_SDcardHandle = osThreadNew(StartTask02, NULL, &write_SDcard_attributes);

  /* creation of i2s_task */
  i2s_taskHandle = osThreadNew(StartTask03, NULL, &i2s_task_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S3;
  PeriphClkInit.I2s3ClockSelection = RCC_I2S3CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_SPI2_InitFast(void)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}


void open_wav(void)
{
    uint8_t pHeader[44];
    uint32_t file_length = 0;;
    uint32_t num_samples = 16000;
    uint32_t i,j;
    int16_t ss[160];
    unsigned int bytesWritten;

    uint32_t SampleRate = 16000ul;
    uint32_t ByteRate = SampleRate * 16ul / 8ul;
    // SampleRate * (BitPerSample/8) * NbrChannels;     /* Number of bytes per second  (sample rate * block align)  */
    // BlockAlign = WaveFormat.NbrChannels * (BitPerSample/8)

    /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
    pHeader[0] = 'R';
    pHeader[1] = 'I';
    pHeader[2] = 'F';
    pHeader[3] = 'F';

    /* Write the file length ---------------------------------------------------*/
    /* The sampling time: this value will be written back at the end of the
     recording operation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
    pHeader[4] = 0x2C;
    pHeader[5] = 0x7D;
    pHeader[6] = 0x00;
    pHeader[7] = 0x00;

    file_length = wav_file_num_samples + 44;
    memcpy(pHeader + 4, &file_length, 4);

    /* Write the file format, must be 'WAVE' -----------------------------------*/
    pHeader[8]  = 'W';
    pHeader[9]  = 'A';
    pHeader[10] = 'V';
    pHeader[11] = 'E';

    /* Write the format chunk, must be'fmt ' -----------------------------------*/
    pHeader[12]  = 'f';
    pHeader[13]  = 'm';
    pHeader[14]  = 't';
    pHeader[15]  = ' ';

    /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
    pHeader[16]  = 0x10;
    pHeader[17]  = 0x00;
    pHeader[18]  = 0x00;
    pHeader[19]  = 0x00;

    /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
    pHeader[20]  = 0x01;
    pHeader[21]  = 0x00;

    /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
    pHeader[22]  = 0x01;//pWaveFormatStruct->NbrChannels;
    pHeader[23]  = 0x00;

    /* Write the Sample Rate in Hz ---------------------------------------------*/
    /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
    pHeader[24]  = (uint8_t)((SampleRate & 0xFF));
    pHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
    pHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
    pHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);

    /* Write the Byte Rate -----------------------------------------------------*/
    pHeader[28]  = (uint8_t)((ByteRate & 0xFF));
    pHeader[29]  = (uint8_t)((ByteRate >> 8) & 0xFF);
    pHeader[30]  = (uint8_t)((ByteRate >> 16) & 0xFF);
    pHeader[31]  = (uint8_t)((ByteRate >> 24) & 0xFF);

    /* Write the block alignment -----------------------------------------------*/
    pHeader[32]  = 2;//pWaveFormatStruct->BlockAlign;
    pHeader[33]  = 0x00;

    /* Write the number of bits per sample -------------------------------------*/
    pHeader[34]  = 16;//pWaveFormatStruct->BitPerSample;
    pHeader[35]  = 0x00;

    /* Write the Data chunk, must be 'data' ------------------------------------*/
    pHeader[36]  = 'd';
    pHeader[37]  = 'a';
    pHeader[38]  = 't';
    pHeader[39]  = 'a';

    /* Write the number of sample data -----------------------------------------*/
    /* This variable will be written back at the end of the recording operation */
    pHeader[40]  = 0x00;
    pHeader[41]  = 0x7D;
    pHeader[42]  = 0x00;
    pHeader[43]  = 0x00;

    memcpy(pHeader + 40, &wav_file_num_samples, 4);


    FRESULT res;

    res = f_open(&wav_file, "mic.wav", FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK)
            while(1);

    //write header
    res = f_write(&wav_file, pHeader, 44, &bytesWritten);
    if(res != FR_OK)
            while(1);
}

void update_wav(uint16_t *buff, uint16_t size)
{
    unsigned int bytesWritten;
    FRESULT res;

    // в байтах в два раза больше
    //wav_file_num_samples += size * 2;

    res = f_write(&wav_file, buff, size * 2, &bytesWritten);
    if(res != FR_OK)
            while(1);

    wav_file_num_samples += bytesWritten;
}

void close_wav(void)
{
    uint8_t pHeader[44];
    uint32_t file_length = 0;;
    uint32_t i,j;
    unsigned int bytesWritten;

    uint32_t SampleRate = 16000ul;
    uint32_t ByteRate = SampleRate * 16ul / 8ul;
    // SampleRate * (BitPerSample/8) * NbrChannels;     /* Number of bytes per second  (sample rate * block align)  */
    // BlockAlign = WaveFormat.NbrChannels * (BitPerSample/8)

    /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
    pHeader[0] = 'R';
    pHeader[1] = 'I';
    pHeader[2] = 'F';
    pHeader[3] = 'F';

    /* Write the file length ---------------------------------------------------*/
    /* The sampling time: this value will be written back at the end of the
     recording operation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
    pHeader[4] = 0x2C;
    pHeader[5] = 0x7D;
    pHeader[6] = 0x00;
    pHeader[7] = 0x00;

    file_length = wav_file_num_samples + 44;
    memcpy(pHeader + 4, &file_length, 4);

    /* Write the file format, must be 'WAVE' -----------------------------------*/
    pHeader[8]  = 'W';
    pHeader[9]  = 'A';
    pHeader[10] = 'V';
    pHeader[11] = 'E';

    /* Write the format chunk, must be'fmt ' -----------------------------------*/
    pHeader[12]  = 'f';
    pHeader[13]  = 'm';
    pHeader[14]  = 't';
    pHeader[15]  = ' ';

    /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
    pHeader[16]  = 0x10;
    pHeader[17]  = 0x00;
    pHeader[18]  = 0x00;
    pHeader[19]  = 0x00;

    /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
    pHeader[20]  = 0x01;
    pHeader[21]  = 0x00;

    /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
    pHeader[22]  = 0x01;//pWaveFormatStruct->NbrChannels;
    pHeader[23]  = 0x00;

    /* Write the Sample Rate in Hz ---------------------------------------------*/
    /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
    pHeader[24]  = (uint8_t)((SampleRate & 0xFF));
    pHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
    pHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
    pHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);

    /* Write the Byte Rate -----------------------------------------------------*/
    pHeader[28]  = (uint8_t)((ByteRate & 0xFF));
    pHeader[29]  = (uint8_t)((ByteRate >> 8) & 0xFF);
    pHeader[30]  = (uint8_t)((ByteRate >> 16) & 0xFF);
    pHeader[31]  = (uint8_t)((ByteRate >> 24) & 0xFF);

    /* Write the block alignment -----------------------------------------------*/
    pHeader[32]  = 2;//pWaveFormatStruct->BlockAlign;
    pHeader[33]  = 0x00;

    /* Write the number of bits per sample -------------------------------------*/
    pHeader[34]  = 16;//pWaveFormatStruct->BitPerSample;
    pHeader[35]  = 0x00;

    /* Write the Data chunk, must be 'data' ------------------------------------*/
    pHeader[36]  = 'd';
    pHeader[37]  = 'a';
    pHeader[38]  = 't';
    pHeader[39]  = 'a';

    /* Write the number of sample data -----------------------------------------*/
    /* This variable will be written back at the end of the recording operation */
    pHeader[40]  = 0x00;
    pHeader[41]  = 0x7D;
    pHeader[42]  = 0x00;
    pHeader[43]  = 0x00;

    memcpy(pHeader + 40, &wav_file_num_samples, 4);


    FRESULT res;


    res = f_lseek(&wav_file, 0);
    //write header
    res = f_write(&wav_file, pHeader, 44, &bytesWritten);
    if(res != FR_OK)
            while(1);

    res = f_close(&wav_file);
    if(res != FR_OK)
        while(1);
}

/*---------------------------------------------------------*/
/* User provided RTC function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called back     */
/* from FatFs module.                                      */

#if !FF_FS_NORTC && !FF_FS_READONLY
DWORD get_fattime (void)
{
	RTCTIME rtc;

	/* Get local time */
	if (!rtc_gettime(&rtc)) return 0;

	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.mday << 16)
			| ((DWORD)rtc.hour << 11)
			| ((DWORD)rtc.min << 5)
			| ((DWORD)rtc.sec >> 1);
}
#endif

void i2s_rx_complete(I2S_HandleTypeDef *hi2s)
{
    static uint8_t cnt = 0;
    uint16_t *_buf_ptr = (uint16_t *)(i2s_buf + 256);

    // в циклическом режиме нужно  менять местами указатели на буфер
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    //osSemaphoreRelease(i2s_readyHandle);

    osMessageQueuePut(i2s_QueueHandle, &_buf_ptr, 0U, 0U);

    if(cnt < 64)
        cnt++;
    else
        HAL_I2S_DMAStop(hi2s);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

void i2s_rx_half_complete(I2S_HandleTypeDef *hi2s)
{
    uint16_t *_buf_ptr = (uint16_t *)i2s_buf;

    // в циклическом режиме нужно  менять местами указатели на буфер
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    //osSemaphoreRelease(i2s_readyHandle);

    osMessageQueuePut(i2s_QueueHandle, &_buf_ptr, 0U, 0U);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

void i2s_rx_stop(I2S_HandleTypeDef *hi2s)
{
    HAL_DMA_Abort_IT(hi2s->hdmarx);
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

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the write_SDcard thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
    uint16_t * recv_ptr = 0;

    rtc_initialize();

    RTCTIME ttime;
    ttime.hour = 18;
    ttime.min = 46;
    ttime.sec = 18;
    ttime.mday = 6;
    ttime.month = 2;
    ttime.year = 2023;
    ttime.wday = 6;

    rtc_settime(&ttime);
#if 1
    // ----------------------------- SD card -----------------------------
    // unselect all SPI devices first
    SDCARD_Unselect();

    // initialize SD-card as fast as possible, it glitches otherwise
    // (this is important only if SPI bus is shared by multiple devices)
    if( SDCARD_Init() != 0)
    {
        while(1);
    }
    else
    {
        MX_SPI2_InitFast();
    }

    // mount the default drive
    res = f_mount(&fs, "", 0);
    if(res != FR_OK)
        while(1);

#endif

    open_wav();

    // надо подождать пока карта завершит свои дела (было записано 44 байта)
    osDelay(70);

    //ready to write
    osSemaphoreRelease(i2s_readyHandle);

    /* Infinite loop */
    for(;;)
    {
        if(osMessageQueueGet(i2s_QueueHandle, &recv_ptr, 0U, 200) == osOK)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

            for(int k = 1; k < 256; k++)
                //recv_ptr[k] = recv_ptr[k*2];
                i2s_buf_s[k] = recv_ptr[k*2];

            //update_wav((uint16_t *)recv_ptr, 256);
            update_wav((uint16_t *)i2s_buf_s, 256);

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }
        else
        {
            close_wav();
            for(;;)
                osDelay(10);
        }
    }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the i2s_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

    //init i2s
    HAL_I2S_RegisterCallback(&hi2s3, HAL_I2S_RX_HALF_COMPLETE_CB_ID, i2s_rx_half_complete);
    HAL_I2S_RegisterCallback(&hi2s3, HAL_I2S_RX_COMPLETE_CB_ID, i2s_rx_complete);


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    //задействуем этот семафор для ождипния задачи с записбю SD
    osSemaphoreAcquire(i2s_readyHandle,  osWaitForever);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    // circular
    HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)i2s_buf, 1024);

    /* Infinite loop */
    for(;;)
        osDelay(1);
  /* USER CODE END StartTask03 */
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

