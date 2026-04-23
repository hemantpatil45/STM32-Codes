/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

// Global structure to hold all sensor data
typedef struct {
    // MPU6050 Data
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    // HMC5883L Data
    int16_t mag_x, mag_y, mag_z;

    // Hall Effect Sensor Data
    uint32_t hall_value;

    // GPS Data
    float latitude;
    float longitude;
} DroneSensorData_t;
// Instantiate the structure
DroneSensorData_t drone_data = {0};
/* USER CODE BEGIN PV */
float kalman_angle = 0.0f;
float kalman_bias = 0.0f;
float P[2][2] = {{0, 0}, {0, 0}};

float updateKalman(float newAngle, float newRate, float dt) {
    // Prediction
    float rate = newRate - kalman_bias;
    kalman_angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + 0.001f);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += 0.003f * dt;

    // Measurement Update
    float S = P[0][0] + 0.03f;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    float y = newAngle - kalman_angle;
    kalman_angle += K[0] * y;
    kalman_bias += K[1] * y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return kalman_angle;
}
/* USER CODE END PV */

// I2C Addresses (Shifted left by 1 for HAL)
#define MPU6050_ADDR (0x68 << 1)
#define HMC5883L_ADDR (0x1E << 1)

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for contole */
osThreadId_t contoleHandle;
const osThreadAttr_t contole_attributes = {
  .name = "contole",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for sensor_read */
osThreadId_t sensor_readHandle;
const osThreadAttr_t sensor_read_attributes = {
  .name = "sensor_read",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for senosr_fusion */
osThreadId_t senosr_fusionHandle;
const osThreadAttr_t senosr_fusion_attributes = {
  .name = "senosr_fusion",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motoroutput */
osThreadId_t motoroutputHandle;
const osThreadAttr_t motoroutput_attributes = {
  .name = "motoroutput",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for sensorData */
osMessageQueueId_t sensorDataHandle;
const osMessageQueueAttr_t sensorData_attributes = {
  .name = "sensorData"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  char debug_msg[] = "DEBUG: UART Transmission is working perfectly!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);


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

  /* Create the queue(s) */
  /* creation of sensorData */
  sensorDataHandle = osMessageQueueNew (30, sizeof(float), &sensorData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of contole */
  contoleHandle = osThreadNew(StartDefaultTask, NULL, &contole_attributes);

  /* creation of sensor_read */
  sensor_readHandle = osThreadNew(StartTask02, NULL, &sensor_read_attributes);

  /* creation of senosr_fusion */
  senosr_fusionHandle = osThreadNew(StartTask03, NULL, &senosr_fusion_attributes);

  /* creation of motoroutput */
  motoroutputHandle = osThreadNew(StartTask04, NULL, &motoroutput_attributes);

  /* creation of communication */
  communicationHandle = osThreadNew(StartTask05, NULL, &communication_attributes);

  /* creation of GPS */
  GPSHandle = osThreadNew(StartTask06, NULL, &GPS_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the contole thread.
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
    osDelay(10000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the sensor_read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	/* USER CODE BEGIN StartTask02 */
	  uint8_t mpu_buf[14];

	  // Note: You must initialize the MPU6050 and HMC5883L before the loop
	  // (e.g., waking up the MPU6050 by writing 0x00 to register 0x6B)
	  uint8_t wakeup = 0x00;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &wakeup, 1, 100);
	  osDelay(10);

	  /* Infinite loop */
	  for(;;)
	  {
	    // 1. Read MPU6050 (Accel & Gyro starting at register 0x3B)
	    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, mpu_buf, 14, 100) == HAL_OK) {
	        drone_data.accel_x = (mpu_buf[0] << 8) | mpu_buf[1];
	        drone_data.accel_y = (mpu_buf[2] << 8) | mpu_buf[3];
	        drone_data.accel_z = (mpu_buf[4] << 8) | mpu_buf[5];
	        // mpu_buf[6] & [7] is temperature
	        drone_data.gyro_x  = (mpu_buf[8] << 8) | mpu_buf[9];
	        drone_data.gyro_y  = (mpu_buf[10] << 8) | mpu_buf[11];
	        drone_data.gyro_z  = (mpu_buf[12] << 8) | mpu_buf[13];
	    }

	    // 2. Read HMC5883L (Magnetometer starting at register 0x03)
	   // if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, mag_buf, 6, 100) == HAL_OK) {
	     //   drone_data.mag_x = (mag_buf[0] << 8) | mag_buf[1];
	       // drone_data.mag_z = (mag_buf[2] << 8) | mag_buf[3]; // Note: HMC5883L order is X, Z, Y
	        //drone_data.mag_y = (mag_buf[4] << 8) | mag_buf[5];
	   // }

	    // 3. Read Hall Effect Sensor (ADC)
	   // HAL_ADC_Start(&hadc1);
	    //if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	       // drone_data.hall_value = HAL_ADC_GetValue(&hadc1);
	    //}
	    //HAL_ADC_Stop(&hadc1);

	    // Run this thread at roughly 100Hz (10ms delay)
	    osDelay(10);
	    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the senosr_fusion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  float dt = 0.01f; // 100Hz timing
  float roll_to_send;

  for(;;)
  {
    // Convert Raw Accel to Degrees
    float accY = drone_data.accel_y / 16384.0f;
    float accZ = drone_data.accel_z / 16384.0f;
    float angle_acc = atan2(accY, accZ) * 57.295f;

    float gyro_rate = drone_data.gyro_x / 131.0f;

    // Run Kalman and capture the result
    roll_to_send = updateKalman(angle_acc, gyro_rate, dt);

    // PUSH to Queue: Send the float to Task 04
    osMessageQueuePut(sensorDataHandle, &roll_to_send, 0, 10);

    osDelay(10); // Maintain 100Hz frequency
  }
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the motoroutput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */

  // 1. Initialize PID variables
  float integral = 0.0f;
  float last_error = 0.0f;
  float error = 0.0f;
  float derivative = 0.0f;
  float pid_out = 0.0f;

  // PID Constants - These are starting points, you will likely need to tune them!
  float kp = 1.25f;
  float ki = 0.02f;
  float kd = 0.45f;

  // Base throttle for hovering (1000 is idle, 2000 is full power)
  // Warning: Start low (e.g., 1200) for safety during testing!
  uint16_t base_hover_throttle = 1450;

  // 2. Start all 4 PWM Channels on Timer 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* Infinite loop */
  for(;;)
  {
    // 3. Calculate PID Error (Target is 0.0 degrees for level hover)
    // We use the global 'kalman_angle' updated by Task 03
    error = 0.0f - kalman_angle;

    // 4. Integral calculation with 'Anti-Windup' clamp
    integral += error * 0.01f; // 0.01f because task runs at 100Hz (10ms)
    if(integral > 400.0f)  integral = 400.0f;
    if(integral < -400.0f) integral = -400.0f;

    // 5. Derivative calculation
    derivative = (error - last_error) / 0.01f;

    // 6. Compute total PID output
    pid_out = (kp * error) + (ki * integral) + (kd * derivative);
    last_error = error;

    // 7. Motor Mixing Logic (X-Configuration)
    // m1: Front-Right, m2: Front-Left, m3: Rear-Left, m4: Rear-Right
    // This logic assumes pid_out is correcting for ROLL.
    int16_t m1 = (int16_t)(base_hover_throttle + pid_out);
    int16_t m2 = (int16_t)(base_hover_throttle - pid_out);
    int16_t m3 = (int16_t)(base_hover_throttle - pid_out);
    int16_t m4 = (int16_t)(base_hover_throttle + pid_out);

    // 8. Final Safety Clamps (Crucial!)
    // Ensures we stay within the standard ESC range of 1ms to 2ms
    if(m1 > 2000) m1 = 2000; if(m1 < 1000) m1 = 1000;
    if(m2 > 2000) m2 = 2000; if(m2 < 1000) m2 = 1000;
    if(m3 > 2000) m3 = 2000; if(m3 < 1000) m3 = 1000;
    if(m4 > 2000) m4 = 2000; if(m4 < 1000) m4 = 1000;

    // 9. Update the Hardware Timer Registers
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, m4);

    // 10. Loop timing (100Hz)
    osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */

	  /* USER CODE BEGIN StartTask05 */

	  // Create a buffer to hold the formatted string
	  char tx_buffer[128];

	  /* Infinite loop */
	  for(;;)
	  {
	    // 1. Format the data into a CSV string
	    // Format: AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Hall,Lat,Lon
	    snprintf(tx_buffer, sizeof(tx_buffer),
	             "%d,%d,%d,%d,%d,%d\r\n",
	             drone_data.accel_x, drone_data.accel_y, drone_data.accel_z,
	             drone_data.gyro_x, drone_data.gyro_y, drone_data.gyro_z);


	    // 2. Transmit the formatted string over UART1
	    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);

	    // 3. Wait before sending the next packet.
	    // 20ms delay gives roughly a 50Hz update rate, which is good for simulation.
	    osDelay(20);
	  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the GPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
