/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_lorawan.h"
#include "stm32_lpm.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*!
 * Unique Devices IDs register set ( STM32L4xxx )
 */
#define         ID1                                 ( 0x1FFF7590 )
#define         ID2                                 ( 0x1FFF7594 )
#define         ID3                                 ( 0x1FFF7594 )



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;

RTC_HandleTypeDef hrtc;




#define UID_BASE_ADDR 0x1FFF7A10
void read_device_uid(uint8_t *uid) {
  // Read 12 bytes from UID base address
  for (int i = 0; i < 12; i++) {
      uid[i] = *(volatile uint8_t *)(UID_BASE_ADDR + i);
  }
}


uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
int _read(int file, char *ptr, int len);
int _write(int file, char *ptr, int len);

static HAL_StatusTypeDef MX_RTC_Init(void);
static void MX_TIM2_Init(void);
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  MX_LoRaWAN_Init();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  // /** Enable RTC peripheral clock
  // */
  // __HAL_RCC_RTC_ENABLE();
}

static HAL_StatusTypeDef MX_RTC_Init(void)
{
  

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // // Reset backup domain
  // __HAL_RCC_BACKUPRESET_FORCE();
  // __HAL_RCC_BACKUPRESET_RELEASE();

  // // Ensure LSE is enabled
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  // RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  // {
  //     printf("LSE configuration failed!\n");
  //     return HAL_ERROR;
  // }

  // // Enable RTC clock
  // __HAL_RCC_RTC_ENABLE();

  // // Disable RTC write protection
  
  // hrtc.Instance->WPR = 0xCA;
  // hrtc.Instance->WPR = 0x53;

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    printf("RTC initialization failed!\n");
    return HAL_ERROR;
  }

  // // Check backup register to avoid overwriting time/date
  // if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
  // {
    /** Initialize RTC and set the Time and Date
    */
    sTime.Hours = 0x11; // 11 in BCD
    sTime.Minutes = 0x59; // 59 in BCD
    sTime.Seconds = 0x00; // 0 in BCD
    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      printf("RTC set time failed!\n");
      return HAL_ERROR;
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x01; // 1 in BCD
    sDate.Year = 0x25; // 25 in BCD

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      printf("RTC set date failed!\n");
      return HAL_ERROR;
    }

  //   // Write backup register
  //   HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
  // }

  printf("RTC initialized successfully!\n");
  return HAL_OK;
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83999; // 84 MHz / (83999 + 1) = 1 kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0; // 1 ms interrupt
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

 /* Configure GPIO pin : DIO1 (PC13) as interrupt */
    GPIO_InitStruct.Pin = DIO1; // GPIO_PIN_13
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Falling edge (high to low)
    GPIO_InitStruct.Pull = GPIO_NOPULL; // External pull-up exists on Nucleo
    HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 as NSS Pin */
  GPIO_InitStruct.Pin = LD2_Pin|NSS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 as Busy Pin*/
  GPIO_InitStruct.Pin = BUSY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  /*Configure GPIO pins : RESET_PIN PA3 as RESET Pin */
  GPIO_InitStruct.Pin = RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_PIN_PORT, &GPIO_InitStruct);


  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, NSS_PIN, GPIO_PIN_SET);

  /* Enable NVIC interrupt for EXTI15_10 (PC13 uses EXTI line 13) */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0); // Priority 1, sub-priority 0
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);



/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

FILE __stdout;
FILE __stdin;
FILE __stderr;

/* Called by C library console/file input
* This function echoes the character received. * If the character is
'\r', it is substituted by '\n'. */
/* Function to send a character to USART2 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
    return len;
}


int _read(int file, char *ptr, int len)
 {
    int i = 0;
    uint8_t ch;
    while (i < len - 1) { // Leave space for null terminator
        HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY); // Blocking read

        // Echo back all characters (including Enter for visibility)
        HAL_UART_Transmit(&huart2, &ch, 1, 1000);

        // Stop on Enter, but don’t include it in the buffer
        if (ch == '\r' || ch == '\n') {
            ptr[i] = '\0'; // Null-terminate at current position
            if (ch == '\r') {
                // If \r, check for \n and consume it (common in terminals)
                HAL_UART_Receive(&huart2, &ch, 1, 10); // Short timeout
                if (ch == '\n') {
                    HAL_UART_Transmit(&huart2, &ch, 1, 1000); // Echo \n
                }
            }
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 1000); // Newline for terminal
            return i; // Return number of actual chars (excluding \r\n)
        }

        ptr[i++] = ch; // Add character to buffer and increment
    }

    ptr[i] = '\0'; // Null-terminate if max length reached
    return i;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
 void Error_Handler(void)
 {
   /* USER CODE BEGIN Error_Handler_Debug */
   printf("Error occurred! Entering safe loop...\n");
 
   while (1)
   {
     HAL_Delay(1000);  // SysTick must be active
     printf("Still in error state...\n");
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