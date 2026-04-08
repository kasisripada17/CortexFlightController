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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Private variables ---------------------------------------------------------*/
uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured[4] = {0,0,0,0};  // Flags for 4 channels
uint32_t Pulse_Width[4] = {0,0,0,0};      // Resulting 1000-2000us values


/* User constants for the 50cm aircraft */
#define RC_MIN 1000
#define RC_MAX 2000


#define LSM6DS3_ADDR_WHO_AM_I  0x0F
#define LSM6DS3_WHO_AM_I_VAL   0x69

#define CS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void update_motors(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) ;
void IMU_Write_Reg(uint8_t reg, uint8_t value) ;

uint8_t IMU_Read_Reg(uint8_t reg_addr) ;

typedef struct {
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t acc_x, acc_y, acc_z;
} IMU_Data_t;

volatile IMU_Data_t raw_sensor_data;
volatile uint8_t imu_data_ready = 0;
volatile uint8_t sensor_data_read = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
uint8_t USB_transmit_buffer[256];
uint16_t transmit_size = 0;
uint8_t IMU_Init(void) ;

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* Start Input Capture for all 4 channels on TIM1 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // PE9
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2); // PE11
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3); // PE13
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); // PE14


  // Enable the Interrupt Line 4 (PC4 uses EXTI4)
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  IMU_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(sensor_data_read)
	  {
		  sensor_data_read = 0;
		  double angX = raw_sensor_data.gyro_x * 0.0175f;
		  double angY = raw_sensor_data.gyro_y * 0.0175f;
		  double angZ = raw_sensor_data.gyro_z * 0.0175f;



		  double accX = raw_sensor_data.acc_x * 0.000244f;
		  double accY = raw_sensor_data.acc_y * 0.000244f;
		  double accZ = raw_sensor_data.acc_z * 0.000244f;

//		  transmit_size = sprintf((char*)USB_transmit_buffer, "\r\n%, %lu, %lu, %lu",
//		 				Pulse_Width[0],
//		 				Pulse_Width[1],
//		 				Pulse_Width[2],
//		 				Pulse_Width[3]);
		  transmit_size = sprintf((char*)USB_transmit_buffer, "\r\n%f, %f, %f, %f, %f, %f",
				 				angX,
				 				angY,
								angZ,
								accX,
								accY,
								accZ);
		 	 CDC_Transmit_HS(USB_transmit_buffer,transmit_size);
	  }


	  update_motors(Pulse_Width[0], Pulse_Width[0] ,Pulse_Width[0],Pulse_Width[0]) ;

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_04DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 274;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 274;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : int_Pin */
  GPIO_InitStruct.Pin = int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(int_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        uint32_t channel_index = 0;
        uint32_t active_channel = 0;

        // Identify which channel triggered the interrupt
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { channel_index = 0; active_channel = TIM_CHANNEL_1; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { channel_index = 1; active_channel = TIM_CHANNEL_2; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { channel_index = 2; active_channel = TIM_CHANNEL_3; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { channel_index = 3; active_channel = TIM_CHANNEL_4; }

        if (Is_First_Captured[channel_index] == 0) // First edge (Rising)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, active_channel);
            Is_First_Captured[channel_index] = 1;

            // Switch polarity to look for the Falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, active_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else // Second edge (Falling)
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, active_channel);

            if (IC_Val2 > IC_Val1)
            {
                Pulse_Width[channel_index] = IC_Val2 - IC_Val1;
            }
            else if (IC_Val1 > IC_Val2) // Handle Timer Overflow
            {
                Pulse_Width[channel_index] = (0xFFFF - IC_Val1) + IC_Val2;
            }

            Is_First_Captured[channel_index] = 0;

            // Reset polarity to Rising edge for next pulse
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, active_channel, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}


void IMU_Write_Reg(uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
    HAL_SPI_Transmit(&hspi1, &value, 1, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

uint8_t IMU_Read_Reg(uint8_t reg_addr) {
    uint8_t command = reg_addr | 0x80; // Set MSB to 1 for Read operation
    uint8_t read_val = 0;

    // 1. Pull CS Low to select the LSM6DS3
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    // 2. Send the register address
    HAL_SPI_Transmit(&hspi1, &command, 1, 10);

    // 3. Receive the register data
    HAL_SPI_Receive(&hspi1, &read_val, 1, 10);

    // 4. Pull CS High to end the transaction
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    return read_val;
}

void update_motors(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m4);
}

uint8_t IMU_Init(void) {
    uint8_t whoAmI = 0;

    // 1. Check Communication
    uint8_t reg = LSM6DS3_ADDR_WHO_AM_I | 0x80; // Read bit
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
    HAL_SPI_Receive(&hspi1, &whoAmI, 1, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    transmit_size = sprintf((char*)USB_transmit_buffer, "\r\nIMU init");
   	 CDC_Transmit_HS(USB_transmit_buffer,transmit_size);

    // 1. Verify Communication (Expects 0x69)
	if (IMU_Read_Reg(0x0F) != 0x69)
	{
		while(1)
		{
	    transmit_size = sprintf((char*)USB_transmit_buffer, "\r\nlsm6ds3 comm fail");
		}
	   	 CDC_Transmit_HS(USB_transmit_buffer,transmit_size);
		return 0;
	}
    transmit_size = sprintf((char*)USB_transmit_buffer, "\r\nlsm6ds3 comm success");
   	 CDC_Transmit_HS(USB_transmit_buffer,transmit_size);
    // 2. Software Reset (Recommended for clean state)
    IMU_Write_Reg(0x12, 0x05); // SW_RESET=1, IF_INC=1 (Auto-increment for burst reads)
    HAL_Delay(10);             // Wait for reboot

    // 3. Accelerometer Config: 1.66 kHz ODR, 8g Full Scale
    IMU_Write_Reg(0x10, 0x8C); // [1000 1100]

    // 4. Gyroscope Config: 1.66 kHz ODR, 500 dps Full Scale
    IMU_Write_Reg(0x11, 0x84); // [1000 0100]

    // 5. Hardware Filtering (Crucial for Hard-Mount noise)
    IMU_Write_Reg(0x13, 0x80); // Enable LPF2 (Secondary digital low-pass filter)
    IMU_Write_Reg(0x17, 0x00); // Set Accel LPF2 cutoff to ODR/50 (approx 33Hz)

    // 6. Interrupt Mapping (Syncs your 1.66 kHz PID loop)
    IMU_Write_Reg(0x0D, 0x02); // Route Gyro Data Ready to INT1 pin (PC4)

    // 7. Global Config
    IMU_Write_Reg(0x12, 0x44); // BDU=1 (Block Data Update) + IF_INC=1

    return 1; // Success
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) { // PC4 triggered

        // 1. Start SPI Burst Read (12 bytes)
        // Address 0x22 (GyroX_L) | 0x80 (Read Bit)
        uint8_t reg = 0x22 | 0x80;
        uint8_t buffer[12];

        // Manual CS Low
        CS_GPIO_Port->BSRR = (uint32_t)CS_Pin << 16;

        // Send address and receive 12 bytes
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Receive(&hspi1, buffer, 12, 10);

        // Manual CS High
        CS_GPIO_Port->BSRR = CS_Pin;

        // 2. Reconstruct 16-bit signed integers (Little Endian)
        raw_sensor_data.gyro_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        raw_sensor_data.gyro_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        raw_sensor_data.gyro_z = (int16_t)((buffer[5] << 8) | buffer[4]);
        raw_sensor_data.acc_x  = (int16_t)((buffer[7] << 8) | buffer[6]);
        raw_sensor_data.acc_y  = (int16_t)((buffer[9] << 8) | buffer[8]);
        raw_sensor_data.acc_z  = (int16_t)((buffer[11] << 8) | buffer[10]);

        sensor_data_read = 1;
        // 3. RUN PID & UPDATE MOTORS
        // This is where you call your PID function immediately
//        run_flight_control_loop();
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
