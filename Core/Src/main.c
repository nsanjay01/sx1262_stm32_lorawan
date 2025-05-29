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






/*Main task implement systime.h and timer.h
 include radio.h and radio.c files*/


// #include "utilities.h"
// #include "region/RegionCommon.h"
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
 * @brief Packet parameters for LoRa packets
 */
// #ifndef LORA_PREAMBLE_LENGTH
// #define LORA_PREAMBLE_LENGTH 8
// #endif
// #ifndef LORA_PKT_LEN_MODE
// #define LORA_PKT_LEN_MODE SX126X_LORA_PKT_EXPLICIT
// #endif
// #ifndef LORA_IQ
// #define LORA_IQ false
// #endif
// #ifndef LORA_CRC
// #define LORA_CRC false
// #endif
// #define PAYLOAD_LENGTH 17

// #define PACKET_TYPE SX126X_PKT_TYPE_LORA


// /*!
//  * @brief LoRa sync word
//  */
// #ifndef LORA_SYNCWORD_PRIVATE_NTW
// #define LORA_SYNCWORD_PRIVATE_NTW   0x12 // 0x12 Private Network

// #endif
// #ifndef LORA_SYNCWORD_PUBLIC_NTW
// #define LORA_SYNCWORD_PUBLIC_NTW    0x34 // 0x34 Public Network

// #endif
// #ifndef LORA_SYNCWORD
// #define LORA_SYNCWORD LORA_SYNCWORD_PRIVATE_NTW
// #endif



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

sx126x_chip_status_t chip_status;

sx126x_pa_cfg_params_t PaCfgParams;

TIM_HandleTypeDef htim2;

RTC_HandleTypeDef hrtc;


/*!
 * User application data
 */
// static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
// static LmHandlerAppData_t AppData =
// {
//     .Buffer = AppDataBuffer,
//     .BufferSize = 0,
//     .Port = 0,
// };


/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

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
uint8_t compute_lora_ldro( const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw );
void sx126x_received( void*, uint8_t*, uint8_t*, uint8_t );
static HAL_StatusTypeDef MX_RTC_Init(void);
static void MX_TIM2_Init(void);


// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(void);
static uint32_t timers_init(void);


// APP_TIMER_DEF(lora_tx_timer_id);                                              ///< LoRa tranfer timer instance.
TimerEvent_t appTimer;														  ///< LoRa tranfer timer instance.
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.



/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = { read_device_uid, BoardGetRandomSeed,
										lorawan_rx_handler, lorawan_has_joined_handler,
										lorawan_confirm_class_handler, lorawan_join_failed_handler};

uint8_t nodeDeviceEUI[8] = {0x00, 0x95, 0x64, 0x1F, 0xDA, 0x91, 0x19, 0x0B};

uint8_t nodeAppEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x01, 0xE1};

uint8_t nodeAppKey[16] = {0x07, 0xC0, 0x82, 0x0C, 0x30, 0xB9, 0x08, 0x70, 0x0C, 0x0F, 0x70, 0x06, 0x00, 0xB0, 0xBE, 0x09};

uint32_t nodeDevAddr = 0x260116F8;

uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};

uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};


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

  // // Retry RTC initialization
  // for (int i = 0; i < 3; i++)
  // {
  //     if (MX_RTC_Init() == HAL_OK)
  //     {
  //         printf("RTC init succeeded on attempt %d!\n", i + 1);
  //         break;
  //     }
  //     printf("RTC init attempt %d failed, retrying...\n", i + 1);
  //     HAL_Delay(1000);
  //     __HAL_RCC_BACKUPRESET_FORCE();
  //     __HAL_RCC_BACKUPRESET_RELEASE();
  // }

  /* USER CODE BEGIN 2 */

  RTC_TimeTypeDef sTime ={0};
  RTC_DateTypeDef sDate = {0};

  
  radio_context_t* context = radio_board_get_radio_context_reference();

  context->spi = hspi2;
  context->nss.GPIO_PORT = GPIOA;
  context->nss.pin = NSS_PIN;
  context->busy.GPIO_PORT = GPIOA;
  context->busy.pin = BUSY_PIN;
  context->reset.pin = RESET_PIN;
  context->reset.GPIO_PORT = RESET_PIN_PORT;

  printf("=====================================\n");
  printf("SX126x LoRaWan test\n");
  printf("=====================================\n");

  sx126x_hal_reset(NULL); // passing null as we don't need pin as it is directly defined.
  // Initialize transmission periodicity variable
  TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

  if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
      {
          printf( "LoRaMac wasn't properly initialized\n" );
          // Fatal error, endless loop.
          while ( 1 )
          {
          }
      }
  // Set system maximum tolerated rx error in milliseconds
  LmHandlerSetSystemMaxRxError(20);
  // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
  LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

  LmHandlerJoin( );

  StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );

  


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Processes the LoRaMac events
    LmHandlerProcess( );

    // Process application uplinks management
    UplinkProcess( );

    __disable_irq();
        if( IsMacProcessPending == 1 )
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            // The MCU wakes up through events
            // BoardLowPowerHandler( );
        }
        __enable_irq();
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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




/* EXTI Callback Function ----------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // if(GPIO_Pin == DIO1)
  // {
  //     sx126x_irq_mask_t irq_regs;
  //   sx126x_get_and_clear_irq_status( &context, &irq_regs );

  //  if( ( irq_regs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
  //  {
  //        printf("[IRQ] TX DONE;\n");
  //        printf("[IRQ] Clearing the TX DONE IRQ\n");
  //  }
  //  else if (( irq_regs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT)
  //  {
  //         printf("[IRQ] TX TIMEOUT;\n");
  //        printf("[IRQ] Clearing the TIMEOUT IRQ\n");
  //  }
  //  else if ((irq_regs & SX126X_IRQ_RX_DONE) == SX126X_IRQ_RX_DONE)
  //  {
  //     uint8_t buffer_rx[255];
  //     uint8_t size;
  //     sx126x_received( &context, buffer_rx, &size, 255 );

  //  }
  // }
}


static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}


static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );

    switch( appData->Port )
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
        {
            // AppLedStateOn = appData->Buffer[0] & 0x01;
        }
        break;
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
    // GpioWrite( &Led2, 1 );
    // TimerStart( &Led2Timer );
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_CONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            // TimerStart( &LedBeaconTimer );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            // TimerStop( &LedBeaconTimer );
            break;
        }
        default:
        {
            break;
        }
    }

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif


/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset( );
    CayenneLppAddDigitalInput( channel++, 1 );
    CayenneLppAddAnalogInput( channel++, (3.1 ) * 100 / 254 );

    CayenneLppCopy( AppData.Buffer );
    AppData.BufferSize = CayenneLppGetSize( );

    if( LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS )
    {
        // Switch LED 1 ON
        // GpioWrite( &Led1, 1 );
        // TimerStart( &Led1Timer );
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}


static void UplinkProcess( void )
{
    uint8_t isPending = 0;
    __disable_irq();
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    __enable_irq();
    if( isPending == 1 )
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
    }

    // Update timer periodicity
    TimerStop( &TxTimer );
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}


/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

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
    printf("Error occurred! Continuing...\n");
    HAL_Delay(1000);
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