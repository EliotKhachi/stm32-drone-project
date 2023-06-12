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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75 // WHO_AM_I register (contains device address)
#define PWR_MGMT_1_REG 0x6B // power management register (write 0 to wake up)
#define SMPLRT_DIV_REG 0x19 // sample rate register
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_REG 0x43
#define TEMP_OUT_H_REG 0x41
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


HAL_StatusTypeDef MPU6050_Read_Accel(float accel[3])
{
	HAL_StatusTypeDef ret;
	uint8_t record_data[6];

	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, record_data, 6, 1000);
	if (ret != HAL_OK) return ret;

	int16_t Accel_X_RAW = (int16_t)(record_data[0] << 8 | record_data[1]);
	int16_t Accel_Y_RAW = (int16_t)(record_data[2] << 8 | record_data[3]);
	int16_t Accel_Z_RAW = (int16_t)(record_data[4] << 8 | record_data[5]);

	accel[0] = (-1)*(Accel_X_RAW/16384.0 - 0.00018382);
	accel[1] = (-1)*(Accel_Y_RAW/16384.0 - 0.0339350589);
	accel[2] = (-1)*(Accel_Z_RAW/16384.0 + 0.2539448);

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Gyro(float gyro[3])
{
	HAL_StatusTypeDef ret;
	uint8_t record_data[6];

	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_REG, 1, record_data, 6, 1000);
	if (ret != HAL_OK) return ret;

	int16_t Gyro_X_RAW = (int16_t)(record_data[0] << 8 | record_data[1]);
	int16_t Gyro_Y_RAW = (int16_t)(record_data[2] << 8 | record_data[3]);
	int16_t Gyro_Z_RAW = (int16_t)(record_data[4] << 8 | record_data[5]);

	gyro[0] = Gyro_X_RAW/131.0 - 2.07064843;
	gyro[1] = Gyro_Y_RAW/131.0 + 2.98403939;
	gyro[2] = Gyro_Z_RAW/131.0 + 0.02834033;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Temp(float *temp) {
	HAL_StatusTypeDef ret;
	uint8_t record_data[2];

	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, record_data, 2, 1000);
	if (ret != HAL_OK) return ret;

	int16_t TEMP_RAW = (int16_t) (record_data[0] << 8 | record_data[1]);

	*temp = TEMP_RAW/340 + 36.53- 5.5297699;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Init() {
	// Wake up sensor (and set clock to 8 MHz) by writing 0x00 to PWR_MGMT_1_REG
	HAL_StatusTypeDef ret;

	uint8_t PWR_MGMT_1_DATA = 0x00;
	ret = HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &PWR_MGMT_1_DATA, 1, 1000);
	if (ret != HAL_OK) return ret;

	// Set Sample Rate to 1 kHz
	uint8_t SMPLRT_DIV_DATA = 0x07;
	ret = HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &SMPLRT_DIV_DATA, 1, 1000);
	if (ret != HAL_OK) return ret;
	// Configure Accelerometers (0x00 corresponds to full scale range of +/- 2g)
	uint8_t ACCEL_CONFIG_DATA = 0x00;
	ret = HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &ACCEL_CONFIG_DATA, 1, 1000);
	if (ret != HAL_OK) return ret;
	// Configure Gyroscopes (0x00 corresponds to full scale range of +/- 250 deg/s)
	uint8_t GYRO_CONFIG_DATA = 0x00;
	ret = HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &GYRO_CONFIG_DATA, 1, 1000);
	if (ret != HAL_OK) return ret;

	return ret;
}

void calibrate_MPU6050(float accel[3], float gyro[3], float accel_offset[3], float gyro_offset[3], float accel_mean[3], float gyro_mean[3], float temp, float temp_offset, float temp_mean, int count, uint16_t sample_size) {
	HAL_StatusTypeDef ret;
	while (count <= sample_size + 100) {
			if (count > 100 && count <= sample_size + 100) {
				// READ DATAs
				ret = MPU6050_Read_Accel(accel); // read accel data
				if (ret != HAL_OK) return;
				ret = MPU6050_Read_Temp(&temp); // read temp data
				if (ret != HAL_OK) return;
				ret = MPU6050_Read_Gyro(gyro);// read gyro data
				if (ret != HAL_OK) return;

				// INCREMENT OFFSETS
				for (int i = 0; i < 3; i++) {
					accel_offset[i] += accel[i]; // increment accel offset
					gyro_offset[i] += gyro[i]; // increment gyro offset
				}
				temp_offset += temp; // increment temp offset
			}
			count++;
		}
		// CALCULATE AVERAGES
		for (int i = 0; i < 3; i++) {
			accel_mean[i] = accel_offset[i]/sample_size;
			gyro_mean[i] = gyro_offset[i]/sample_size;
		}
		temp_mean = temp_offset/sample_size;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //uint8_t buf[12];
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Settings for MPU6050

  HAL_StatusTypeDef ret;
  uint8_t check;
  float accel[3];
  float gyro[3];
  float accel_offset[3] = {0, 0, 0};
  float gyro_offset[3] = {0, 0, 0};
  float accel_mean[3];
  float gyro_mean[3];

  float temp;
  float temp_offset = 0;
  float temp_mean;

  uint16_t sample_size = 1000;
  int count = 0;

  // Sanity check (Check that "check" = 0x68)
  ret = HAL_I2C_Mem_Read (&hi2c1,MPU6050_ADDR | 0x01, WHO_AM_I_REG, 1, &check, 1, 1000);
  if (ret != HAL_OK) return ret;
  ret = MPU6050_Init(MPU6050_ADDR, PWR_MGMT_1_REG, SMPLRT_DIV_REG, ACCEL_CONFIG_REG, GYRO_CONFIG_REG);
  if (ret != HAL_OK) return ret;

  /* USER CODE END 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
