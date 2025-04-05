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
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE SX126X_LORA_PKT_EXPLICIT
#endif
#ifndef LORA_IQ
#define LORA_IQ false
#endif
#ifndef LORA_CRC
#define LORA_CRC false
#endif
#define PAYLOAD_LENGTH 17

#define PACKET_TYPE SX126X_PKT_TYPE_LORA


/*!
 * @brief LoRa sync word
 */
#ifndef LORA_SYNCWORD_PRIVATE_NTW
#define LORA_SYNCWORD_PRIVATE_NTW   0x12 // 0x12 Private Network

#endif
#ifndef LORA_SYNCWORD_PUBLIC_NTW
#define LORA_SYNCWORD_PUBLIC_NTW    0x34 // 0x34 Public Network

#endif
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_SYNCWORD_PRIVATE_NTW
#endif



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

sx126x_chip_status_t chip_status;

Hal_context context;

sx126x_pa_cfg_params_t PaCfgParams;

TIM_HandleTypeDef htim2;

RTC_HandleTypeDef hrtc;









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
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);




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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  RTC_TimeTypeDef sTime ={0};
  RTC_DateTypeDef sDate = {0};


  context.spi = hspi2;
  context.nss.GPIO_PORT = GPIOA;
  context.nss.pin = NSS_PIN;
  context.busy.GPIO_PORT = GPIOA;
  context.busy.pin = BUSY_PIN;



  /*Setting Pa cfg Params for +14dBm */
  PaCfgParams.pa_duty_cycle = 0x02;
  PaCfgParams.hp_max = 0x02;
  PaCfgParams.device_sel = 0x00;
  PaCfgParams.pa_lut = 0x01;



  /*Setting Lora params*/

  static sx126x_mod_params_lora_t lora_mod_params = {
    .sf   = SX126X_LORA_SF7,
    .bw   = SX126X_LORA_BW_125,
    .cr   = SX126X_LORA_CR_4_5,
    .ldro = 0,  // Will be initialized during radio init
  };


  /* Setting Lora packet params*/

  const sx126x_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .header_type          = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes     = PAYLOAD_LENGTH,
    .crc_is_on            = LORA_CRC,
    .invert_iq_is_on      = LORA_IQ,
  };


  /*DATA*/
  uint8_t buffer_tx[PAYLOAD_LENGTH] = "hello from stm32";

  /*RF switch*/
  const bool dio2_is_set_as_rf_switch = true;

  /*TXCO parameters*/
  typedef struct smtc_shield_sx126x_xosc_cfg_s
  {
    bool                        tcxo_is_radio_controlled;
    sx126x_tcxo_ctrl_voltages_t supply_voltage;
    uint32_t                    startup_time_in_tick;
  } smtc_shield_sx126x_xosc_cfg_t;
  smtc_shield_sx126x_xosc_cfg_t tcxo_params ={.startup_time_in_tick =500, .supply_voltage= SX126X_TCXO_CTRL_1_7V };



  printf("STM32 started\n");
  char stm32_input_char[10];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Prompt user for a character
	          printf("STM32: Enter a character to send to ESP32: ");
	          fflush(stdout);

	          // Read a single character from UART
	          int chars_read = _read(0, stm32_input_char, 8); // Read 1 char + null terminator
	          if (chars_read > 0) {


              printf("[INFO]Resetting the SX1262\n");
              ASSERT_SX126X_RC( sx126x_reset(&context ) );

              printf("[INFO] Enabling Retention in SX1262\n");
              ASSERT_SX126X_RC( sx126x_init_retention_list( &context ) );

              printf("[INFO] Setting RF switch\n");
              ASSERT_SX126X_RC( sx126x_set_dio2_as_rf_sw_ctrl( &context, dio2_is_set_as_rf_switch ) );

              printf("[INFO] Setting TXCO switch\n");
              ASSERT_SX126X_RC( sx126x_set_dio3_as_tcxo_ctrl( &context,tcxo_params.supply_voltage,tcxo_params.startup_time_in_tick ) );

              printf("[INFO] Calibrating SX1262\n");
              ASSERT_SX126X_RC( sx126x_cal( &context, SX126X_CAL_ALL ) );
              
              if(SX126X_STATUS_OK == sx126x_get_status(&context, &chip_status ))
                  {
                    printf("[INFO]commad status = %d", chip_status.cmd_status);
                    if(chip_status.chip_mode == SX126X_CHIP_MODE_STBY_RC)
                    {
                      printf("[INFO]chip mode = %d\n", chip_status.chip_mode);
                      printf("[INFO]Chip mode to STDBY_RC\n");

                    }
                    else
                    {
                      printf("[INFO]chip mode is different i.e %d\n", chip_status.chip_mode);
                      printf("[INFO]setting chip mode to STDBY_RC\n");
                      if(SX126X_STATUS_OK == sx126x_set_standby(&context, SX126X_STANDBY_CFG_RC))
                      {
                      printf("[INFO]set to STDBY_RC\n");
                      }
                    }
                  }
              printf("[INFO]Setting the packet type to Lora\n");
              ASSERT_SX126X_RC(sx126x_set_pkt_type( &context, SX126X_PKT_TYPE_LORA));
              
              printf("[INFO]Setting the frequency to 869000000 Hz\n");
              ASSERT_SX126X_RC(sx126x_set_rf_freq(&context, (uint32_t)868000000));
             
              printf("[INFO] Setting the Pa configs for +14 dBm\n");
              ASSERT_SX126X_RC(sx126x_set_pa_cfg(&context, &PaCfgParams ));

              printf("[INFO] Setting Output power to +10 dBm \n");
              ASSERT_SX126X_RC(sx126x_set_tx_params(&context,10, SX126X_RAMP_40_US ));

              printf("[INFO] Setting Buffer Base Address \n");
              ASSERT_SX126X_RC(sx126x_set_buffer_base_address(&context, 0,0));


              if(chars_read == 1)
              {
                   printf("[INFO] Setting data in the buffer\n");
                   ASSERT_SX126X_RC(sx126x_write_buffer( &context, 0, buffer_tx, PAYLOAD_LENGTH ));
              }
             

              printf("[INFO] Setting Lora Modulations Params\n");
              lora_mod_params.ldro = compute_lora_ldro(SX126X_LORA_SF7,SX126X_LORA_BW_125);
              ASSERT_SX126X_RC(sx126x_set_lora_mod_params(&context, &lora_mod_params));

              printf("[INFO] Setting Lora Packet params\n");
              ASSERT_SX126X_RC(sx126x_set_lora_pkt_params( &context, &lora_pkt_params ) );

              printf("[INFO] Setting the IRQ and mapping to DI01\n");
              ASSERT_SX126X_RC(sx126x_set_dio_irq_params(&context, SX126X_IRQ_ALL,
                                SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR,
                                SX126X_IRQ_NONE, SX126X_IRQ_NONE ));

              printf("[INFO] Setting the SYNC WORD\n");
              ASSERT_SX126X_RC(sx126x_set_lora_sync_word( &context, LORA_SYNCWORD ) );

            if(chars_read == 1)
            {
                  printf("[INFO] Starting TX\n");
                  sx126x_status_t tx_status = sx126x_set_tx(&context, 3000);
                  if (tx_status != SX126X_STATUS_OK) {
                        printf("[ERROR] TX failed: %d\n", tx_status);
                    }
            }
            else
            {
              printf("[INFO] Starting Rx for 10 se\n");
              sx126x_status_t tx_status = sx126x_set_rx(&context, 10000);
                  if (tx_status != SX126X_STATUS_OK) {
                        printf("[ERROR] TX failed: %d\n", tx_status);
                    }
            }
          

        // // Poll BUSY pin to confirm TX is happening
        // printf("BUSY pin state: ");
        // for (int i = 0; i < 10; i++) {
        //     printf("%d ", HAL_GPIO_ReadPin(GPIOA, BUSY_PIN));
        //     HAL_Delay(50); // Wait 500ms total
        // }
        // printf("\n");

        // // Poll IRQ status manually
        // sx126x_irq_mask_t irq_reg;
        // for (int i = 0; i < 5; i++) {
        //     sx126x_get_irq_status(&context, &irq_reg);
        //     printf("IRQ after %d00ms: %04X\n", i * 5, irq_reg);
        //     if (irq_reg & SX126X_IRQ_TX_DONE) {
        //         printf("[IRQ] TX DONE detected\n");
        //         sx126x_clear_irq_status(&context, SX126X_IRQ_TX_DONE);
        //         break;
        //     } else if (irq_reg & SX126X_IRQ_TIMEOUT) {
        //         printf("[IRQ] TIMEOUT detected\n");
        //         sx126x_clear_irq_status(&context, SX126X_IRQ_TIMEOUT);
        //         break;
        //     }
            HAL_Delay(500);


          // }
              chars_read = 0;

          }
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
}

static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

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
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 11;
  sTime.Minutes = 59;
  sTime.Seconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if(GPIO_Pin == DIO1)
  {
      sx126x_irq_mask_t irq_regs;
    sx126x_get_and_clear_irq_status( &context, &irq_regs );

   if( ( irq_regs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
   {
         printf("[IRQ] TX DONE;\n");
         printf("[IRQ] Clearing the TX DONE IRQ\n");
   }
   else if (( irq_regs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT)
   {
          printf("[IRQ] TX TIMEOUT;\n");
         printf("[IRQ] Clearing the TIMEOUT IRQ\n");
   }
   else if ((irq_regs & SX126X_IRQ_RX_DONE) == SX126X_IRQ_RX_DONE)
   {
      uint8_t buffer_rx[255];
      uint8_t size;
      sx126x_received( &context, buffer_rx, &size, 255 );

   }
  }
}



uint8_t compute_lora_ldro( const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw )
{
    switch( bw )
    {
    case SX126X_LORA_BW_500:
        return 0;

    case SX126X_LORA_BW_250:
        if( sf == SX126X_LORA_SF12 )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_125:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_062:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_041:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) ||
            ( sf == SX126X_LORA_SF9 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_031:
    case SX126X_LORA_BW_020:
    case SX126X_LORA_BW_015:
    case SX126X_LORA_BW_010:
    case SX126X_LORA_BW_007:
        return 1;

    default:
        return 0;
    }
}



void sx126x_received( void* context, uint8_t* buffer, uint8_t* size, uint8_t max_size )
{
  sx126x_rx_buffer_status_t rx_buffer_status;
  sx126x_pkt_status_lora_t  pkt_status_lora;

  sx126x_get_rx_buffer_status( context, &rx_buffer_status );

  if( max_size < rx_buffer_status.pld_len_in_bytes )
    {
        printf( "Received more bytes than expected (%d vs %d), reception in buffer cancelled.\n",
                             rx_buffer_status.pld_len_in_bytes, max_size );
        *size = 0;
    }
    else
    {
        sx126x_read_buffer( context, rx_buffer_status.buffer_start_pointer, buffer, rx_buffer_status.pld_len_in_bytes );
        *size = rx_buffer_status.pld_len_in_bytes;
    }
    // Copy to null-terminated string
    char str[((*size) + 1)];
    memcpy(str, buffer, *size);
    str[*size] = '\0';
            
    printf("[RX] Data: %s \nPayload_length: %d\n", str, rx_buffer_status.pld_len_in_bytes);
    printf( "Packet status:\n" );
    if( PACKET_TYPE == SX126X_PKT_TYPE_LORA )
    {
        sx126x_get_lora_pkt_status( context, &pkt_status_lora );
        printf( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        printf( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
        printf( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
    }
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
