/* USER CODE BEGIN Header */
#include <stdbool.h>
#include<stdio.h>
/* USER CODE END  Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/*hemant maruti  patil*/
/*last updated 18 april */
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum
{
   CMD_RESET_OBC  = 0x11,
   CMD_NO_COMMAND = 0X0E,
   CMD_TIME_STAMP_SYSTEM = 0x0D,
   CMD_FPGA_ON      =0x01,
   CMD_FRONTEND_ON   =0x02,
   CMD_ANTENNA_DEPLOY_ON   = 0x03,
   CMD_ANTENNA_DEPLOY_OFF = 0x04,
   CMD_TIME_STAMP_FPGA = 0x05,
   CMD_FPGA_OFF = 0x06,
   CMD_FRONTEND_OFF  = 0x07,
   CMD_START_HOUSEKEEPING = 0x08,
   CMD_DOWNLOAD_HOUSEKEEPING_DATA  = 0x09,
   CMD_DOWNLOAD_FPGA_DATA  = 0x0A,
   CMD_RESET_HOUSEKEEPING_MEMORY  = 0x0B,
	CMD_RESET_FPGADATA_MEMORY = 0x0C,
} command_t;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
   uint32_t timestamp;  // 4 bytes
   uint16_t VoltPos;    // 2 bytes
   uint16_t VoltNeg;    // 2 bytes
   uint16_t Current;    // 2 bytes
   uint16_t Temp1;      // 2 bytes
   uint16_t Temp2;      // 2 bytes
   uint16_t Temp3;      // 2 bytes
   uint32_t crc;        // 4 bytes
} __attribute__((packed)) SensorLog_t;
typedef struct {
   uint8_t cmd;     // command
   uint8_t flag;   // flag for housekeeping data
} Message_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STORAGE_START_ADDR_7  0x08060000  // Base address of Sector 7
#define STORAGE_SECTOR_SIZE   (128 * 1024) // 128 Kbytes
#define STORAGE_END_ADDR_7    (STORAGE_START_ADDR_7 + STORAGE_SECTOR_SIZE)
// Definition of GPIO PIN Functions
#define FPGA_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define FPGA_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define FRONTEND_ON() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
#define FRONTEND_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define ANTENNA_DEPLOY_ON() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
#define ANTENNA_DEPLOY_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
/* USER CODE BEGIN PV */
uint32_t Get_Next_Empty_Address_7(void);
uint32_t AdressHousekeeping = 0x08060000;
uint32_t currentFlashAddress_7;
SensorLog_t HousekeepingLog;
uint8_t cmd_byte;        // For the 1-byte command
uint8_t FPGA_Byte;
uint8_t time_data[4];    // For the 4-byte timestamp
uint32_t PS4_time=0;       // Your final 32-bit time
bool waiting_for_time = false;
uint8_t txData[] = "Telemetry command recieved\r\n";

uint16_t adcBuffer[6];   // DMA fills this in scan order


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for Telementrytask */
osThreadId_t TelementrytaskHandle;
const osThreadAttr_t Telementrytask_attributes = {
  .name = "Telementrytask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for housekeeping */
osThreadId_t housekeepingHandle;
const osThreadAttr_t housekeeping_attributes = {
  .name = "housekeeping",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Command_Execute */
osThreadId_t Command_ExecuteHandle;
const osThreadAttr_t Command_Execute_attributes = {
  .name = "Command_Execute",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timer_flag */
osMessageQueueId_t timer_flagHandle;
const osMessageQueueAttr_t timer_flag_attributes = {
  .name = "timer_flag"
};
/* Definitions for command */
osMessageQueueId_t commandHandle;
const osMessageQueueAttr_t command_attributes = {
  .name = "command"
};
/* Definitions for FPGA_Command */
osMessageQueueId_t FPGA_CommandHandle;
const osMessageQueueAttr_t FPGA_Command_attributes = {
  .name = "FPGA_Command"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};


/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
void StartTelementrytask(void *argument);
void Starthousekeeping(void *argument);
void StartCommand_Execute(void *argument);

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
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6, &cmd_byte, 1);
  HAL_UART_Receive_IT(&huart1, &FPGA_Byte, 1);
   currentFlashAddress_7 = Get_Next_Empty_Address_7();



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of timer_flag */
  timer_flagHandle = osMessageQueueNew (16, sizeof(uint16_t), &timer_flag_attributes);

  /* creation of command */
  commandHandle = osMessageQueueNew (16, sizeof(uint16_t), &command_attributes);

  /* creation of FPGA_Command */
  FPGA_CommandHandle = osMessageQueueNew (16, sizeof(uint16_t), &FPGA_Command_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Telementrytask */
  TelementrytaskHandle = osThreadNew(StartTelementrytask, NULL, &Telementrytask_attributes);

  /* creation of housekeeping */
  housekeepingHandle = osThreadNew(Starthousekeeping, NULL, &housekeeping_attributes);

  /* creation of Command_Execute */
  Command_ExecuteHandle = osThreadNew(StartCommand_Execute, NULL, &Command_Execute_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 17856;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */
  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */
  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  /* USER CODE END USART6_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC8 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART6) {

       if (!waiting_for_time) {
           // --- WE JUST RECEIVED THE 1-BYTE COMMAND ---
           if (cmd_byte == 0x0D) {
               // It's the time command!
               waiting_for_time = true;
               // RECONFIGURE: Tell UART to wait for exactly 4 bytes now
               HAL_UART_Receive_IT(&huart6, time_data, 4);
           }


           else {
               // It's a normal command, send to RTOS
           	Message_t Rcommand = {0};
				Rcommand.cmd = cmd_byte;
               osMessageQueuePut(commandHandle, &Rcommand, 0, 0);
               // Stay in 1-byte mode
               HAL_UART_Receive_IT(&huart6, &cmd_byte, 1);
           }
       }
       else {
           // --- WE JUST RECEIVED THE 4-BYTE TIMESTAMP ---
           // Combine the 4 bytes into your 32-bit variable
           PS4_time = (uint32_t)time_data[0] << 24 |
                      (uint32_t)time_data[1] << 16 |
                      (uint32_t)time_data[2] << 8  |
                      (uint32_t)time_data[3];
           waiting_for_time = false;
           // RECONFIGURE: Switch back to 1-byte mode for the next command
           HAL_UART_Receive_IT(&huart6, &cmd_byte, 1);
	         HAL_UART_Transmit(&huart6, (uint8_t*)"TimeStampcomplete\n", 16, HAL_MAX_DELAY);

       }
   }
}



void HAL_UART_R9xCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART1) {
	   HAL_UART_Transmit_DMA(&huart6, FPGA_Byte, 1);

   }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
   if (htim->Instance == TIM2) {
       Message_t timer_trig = {0} ;
       timer_trig.flag =1;
       osMessageQueuePut(timer_flagHandle, &timer_trig, 0, 0);
   }
}

uint32_t Get_Next_Empty_Address_7(void) {
   uint32_t address = STORAGE_START_ADDR_7;
   // Scan word-by-word (incrementing by 4 bytes)
   while (address < STORAGE_END_ADDR_7) {
       // Read the 32-bit word at the current address
       if (*(__IO uint32_t*)address == 0xFFFFFFFF) {
           return address; // Found empty space!
       }
       address += 4;
   }
   return 0; // Sector is completely full
}

void Antenna_Deploy(void){
	FPGA_OFF();
	FRONTEND_OFF();
	ANTENNA_DEPLOY_ON();
	osDelay(120000);
	ANTENNA_DEPLOY_OFF();
   }

void Save_Housekeeping_Log(SensorLog_t *log) {
   // 1. Cast the struct to a 32-bit pointer so we can read it word-by-word
   uint32_t *dataPtr = (uint32_t *)log;
   osMutexAcquire(myMutex01Handle, osWaitForever);
   HAL_FLASH_Unlock();
   for (int i = 0; i < 5; i++) {
       // Calculate the target address for this specific word
       uint32_t targetAddr = currentFlashAddress_7 + (i * 4);
       // Program one word at a time
       if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, targetAddr, dataPtr[i]) != HAL_OK) {
           // Handle error (e.g., flash write failed)
           break;
       }
   }
   currentFlashAddress_7 += sizeof(SensorLog_t);

   HAL_FLASH_Lock();
   osMutexRelease(myMutex01Handle);
}
void Transmitt_Stored_HK(void){
	   osMutexAcquire(myMutex01Handle, osWaitForever);

	 uint32_t startAddr = STORAGE_START_ADDR_7;
	    uint32_t endAddr   = currentFlashAddress_7;

	    // Safety check: nothing to send
	    if (endAddr <= startAddr)
	        return;

	    uint32_t length = endAddr - startAddr;

	    // Transmit flash contents using DMA
	    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)startAddr, length);
	    osMutexRelease(myMutex01Handle);


  }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {

        HousekeepingLog.VoltPos = adcBuffer[0];
        HousekeepingLog.VoltNeg = adcBuffer[1];
        HousekeepingLog.Current = adcBuffer[2];
        HousekeepingLog.Temp1   = adcBuffer[3];
        HousekeepingLog.Temp2   = adcBuffer[4];
        HousekeepingLog.Temp3   = adcBuffer[5];

        HAL_ADC_Stop_DMA(&hadc1);   // since we're NOT using circular mode


    }
}


void EraseFlash_6(void)
{
	   osMutexAcquire(myMutex01Handle, osWaitForever);
  FLASH_EraseInitTypeDef FlashEraseDefination;  // Flash erase configuration
  uint32_t FlashEraseFault = 0;
  HAL_FLASH_Unlock();  // Unlock the Flash for write/erase operations
  FlashEraseDefination.TypeErase     = FLASH_TYPEERASE_SECTORS;
  FlashEraseDefination.Banks         = FLASH_BANK_1;
  FlashEraseDefination.NbSectors     = 1;
  FlashEraseDefination.Sector        = FLASH_SECTOR_6;  // Change to your target sector
  FlashEraseDefination.VoltageRange  = FLASH_VOLTAGE_RANGE_3;  // 2.7V to 3.6V (usually 3.3V)
  HAL_FLASHEx_Erase(&FlashEraseDefination, &FlashEraseFault);
  HAL_FLASH_Lock();  // Lock the Flash again
  osMutexRelease(myMutex01Handle);

}
void EraseFlash_7(void)
{
	   osMutexAcquire(myMutex01Handle, osWaitForever);
  FLASH_EraseInitTypeDef FlashEraseDefination;  // Flash erase configuration
  uint32_t FlashEraseFault = 0;
  HAL_FLASH_Unlock();  // Unlock the Flash for write/erase operations
  FlashEraseDefination.TypeErase     = FLASH_TYPEERASE_SECTORS;
  FlashEraseDefination.Banks         = FLASH_BANK_1;
  FlashEraseDefination.NbSectors     = 1;
  FlashEraseDefination.Sector        = FLASH_SECTOR_7;  // Change to your target sector
  FlashEraseDefination.VoltageRange  = FLASH_VOLTAGE_RANGE_3;  // 2.7V to 3.6V (usually 3.3V)
  HAL_FLASHEx_Erase(&FlashEraseDefination, &FlashEraseFault);
  HAL_FLASH_Lock();  // Lock the Flash again
  osMutexRelease(myMutex01Handle);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTelementrytask */
/* USER CODE END Header_StartTelementrytask */
void StartTelementrytask(void *argument)
{
  /* USER CODE BEGIN 5 */
	 for(;;)
		 {
		 osDelay(30);
		 }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Starthousekeeping */
/* USER CODE END Header_Starthousekeeping */
void Starthousekeeping(void *argument)
{
  /* USER CODE BEGIN Starthousekeeping */

	Message_t received_cmd;
			    osStatus_t status;
	 for(;;)
	 {   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

		 status = osMessageQueueGet(timer_flagHandle, &received_cmd, NULL, osWaitForever);

	      if (status==osOK){

	    	  if (received_cmd.flag == 1) {
	    	      // Corrected length to 7 to include \n\r

	    		  HousekeepingLog.timestamp = PS4_time + (HAL_GetTick() / 1000);

	    		  /* Start ADC once */

	    		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 6);
	    		  osDelay(10);
	    		  char msg[128];
	    		         int len = snprintf(msg, sizeof(msg),
	    		                            "VP:%u VN:%u I:%u T1:%u T2:%u T3:%u\r\n",
	    		                            HousekeepingLog.VoltPos,
	    		                            HousekeepingLog.VoltNeg,
	    		                            HousekeepingLog.Current,
	    		                            HousekeepingLog.Temp1,
	    		                            HousekeepingLog.Temp2,
	    		                            HousekeepingLog.Temp3);

	    		     HAL_UART_Transmit(&huart6, (uint8_t*)msg, len, HAL_MAX_DELAY);

	    		  /* CRC */
	    		  HousekeepingLog.crc =
	    		      HAL_CRC_Calculate(&hcrc,
	    		                        (uint32_t*)&HousekeepingLog,
	    		                        sizeof(HousekeepingLog) / 4);


	    		  Save_Housekeeping_Log(&HousekeepingLog);

	    		  /* Clear command flag */
	    		  received_cmd.flag = 0;


	    	  }
	 }
	 }
  /* USER CODE END Starthousekeeping */
}

/* USER CODE BEGIN Header_StartCommand_Execute */
/* USER CODE END Header_StartCommand_Execute */
void StartCommand_Execute(void *argument)
{
  /* USER CODE BEGIN StartCommand_Execute */
	Message_t received_cmd;
		    osStatus_t status;
		    for(;;) {

		        status = osMessageQueueGet(commandHandle, &received_cmd, NULL, osWaitForever);
		        if (status == osOK) {
		      		 HAL_UART_Transmit(&huart6, txData, sizeof(txData)-1, HAL_MAX_DELAY);

		            switch (received_cmd.cmd) {

		                case CMD_RESET_OBC:
		                	NVIC_SystemReset();
		                    break;
		                case CMD_FPGA_ON:
		                    FPGA_ON();
		                    break;
		                case CMD_FRONTEND_ON:
		          		  HAL_UART_Transmit(&huart6, (uint8_t*)"FON\n\r", 6, 10);
		                    FRONTEND_ON();
		          		  HAL_UART_Transmit(&huart6, (uint8_t*)"FONC\n\r", 6, 10);

		                    break;
		                case CMD_ANTENNA_DEPLOY_ON:
		                	Antenna_Deploy();
		                    break;
		                case CMD_ANTENNA_DEPLOY_OFF:
		                   ANTENNA_DEPLOY_OFF();
		                    break;
		                case CMD_TIME_STAMP_SYSTEM:
		                	break;
		                case CMD_TIME_STAMP_FPGA:
		                    break;
		                case CMD_FPGA_OFF:
		                    FPGA_OFF();
		                    break;
		                case CMD_FRONTEND_OFF:
		                    FRONTEND_OFF();
		                    break;
		                case CMD_START_HOUSEKEEPING:
		                    HAL_TIM_Base_Start_IT(&htim2);

		                    break;
		                case CMD_DOWNLOAD_HOUSEKEEPING_DATA:
		                	Transmitt_Stored_HK();
		                    break;
		                case CMD_DOWNLOAD_FPGA_DATA:

		                    break;
		                case CMD_RESET_HOUSEKEEPING_MEMORY:
		                	EraseFlash_7();
		                    break;
		                case CMD_RESET_FPGADATA_MEMORY:
		                	EraseFlash_6();
		                    break;
		                case CMD_NO_COMMAND:
		                    /* Explicitly ignore or log No Command */
		                    break;
		                default:
		                    /* If UART receives a byte that isn't in your enum (e.g., 0xFF),
		                       this prevents the system from doing something dangerous.
		                    */
		                    // Log_Error("Invalid Command Received");
		                    break;
		            }
		        }
		    }

  /* USER CODE END StartCommand_Execute */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
