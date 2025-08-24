/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart_if.c
  * @author  MCD Application Team
  * @brief   Configuration of UART driver interface for hyperterminal communication
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usart_if.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/**
  * @brief DMA handle
  */
extern DMA_HandleTypeDef hdma_usart2_tx;

/**
  * @brief UART handle
  */
extern UART_HandleTypeDef huart2;

/**
  * @brief buffer to receive 1 character
  */
uint8_t charRx;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Trace driver callbacks handler
  */
const UTIL_ADV_TRACE_Driver_s UTIL_TraceDriver =
{
  vcom_Init,
  vcom_DeInit,
  vcom_ReceiveInit,
  vcom_Trace_DMA,
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief  TX complete callback
  * @return none
  */
static void (*TxCpltCallback)(void *);

/**
  * @brief  RX complete callback
  * @param  rxChar ptr of chars buffer sent by user
  * @param  size buffer size
  * @param  error errorcode
  * @return none
  */
static void (*RxCpltCallback)(uint8_t *rxChar, uint16_t size, uint8_t error);

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
UTIL_ADV_TRACE_Status_t vcom_Init(void (*cb)(void *))
{
  /* USER CODE BEGIN vcom_Init_1 */

  /* USER CODE END vcom_Init_1 */
  TxCpltCallback = cb;
  // Note: MX_DMA_Init() is usually called once in main.c
  // This call is kept for compatibility with the original example structure.
  MX_DMA_Init();
  MX_USART2_UART_Init();
  return UTIL_ADV_TRACE_OK;
  /* USER CODE BEGIN vcom_Init_2 */

  /* USER CODE END vcom_Init_2 */
}

UTIL_ADV_TRACE_Status_t vcom_DeInit(void)
{
  /* USER CODE BEGIN vcom_DeInit_1 */

  /* USER CODE END vcom_DeInit_1 */
  /* ##-1- Reset peripherals ################################################## */
  __HAL_RCC_USART2_FORCE_RESET();
  __HAL_RCC_USART2_RELEASE_RESET();

  /* ##-2- MspDeInit ################################################## */
  HAL_UART_MspDeInit(&huart2);

  /* ##-3- Disable the NVIC for DMA ########################################### */
  /* USER CODE BEGIN 1 */
  // F4 Change: Use the correct DMA Stream IRQn for USART2_TX
  HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  return UTIL_ADV_TRACE_OK;
  /* USER CODE END 1 */
  /* USER CODE BEGIN vcom_DeInit_2 */

  /* USER CODE END vcom_DeInit_2 */
}

void vcom_Trace(uint8_t *p_data, uint16_t size)
{
  /* This function does not need changes */
  HAL_UART_Transmit(&huart2, p_data, size, 1000);
}

UTIL_ADV_TRACE_Status_t vcom_Trace_DMA(uint8_t *p_data, uint16_t size)
{
  /* This function does not need changes */
  HAL_UART_Transmit_DMA(&huart2, p_data, size);
  return UTIL_ADV_TRACE_OK;
}

UTIL_ADV_TRACE_Status_t vcom_ReceiveInit(void (*RxCb)(uint8_t *rxChar, uint16_t size, uint8_t error))
{
  /* USER CODE BEGIN vcom_ReceiveInit_1 */

  /* USER CODE END vcom_ReceiveInit_1 */

  /* Record the callback function */
  RxCpltCallback = RxCb;

  /*
   * The original function was designed to wake the MCU from Stop Mode.
   * That feature is not available on the STM32F4 series.
   * This revised version simply starts a continuous background reception
   * using interrupts. The HAL_UART_RxCpltCallback will be called for
   * each byte received, and it will automatically restart the reception.
   */

  /* Start UART reception in interrupt mode */
  if(HAL_UART_Receive_IT(&huart2, &charRx, 1) != HAL_OK)
  {
    /* Reception start error */
    Error_Handler();
  }

  return UTIL_ADV_TRACE_OK;
  /* USER CODE BEGIN vcom_ReceiveInit_2 */

  /* USER CODE END vcom_ReceiveInit_2 */
}

void vcom_Resume(void)
{
  /* This function does not need changes */
  /*to re-enable lost UART settings*/
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /*to re-enable lost DMA settings*/
  if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* This function does not need changes */
  if(huart->Instance==USART2)
  {
    TxCpltCallback(NULL);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* This function does not need changes */
  if(huart->Instance==USART2)
  {
    if ((NULL != RxCpltCallback) && (HAL_UART_ERROR_NONE == huart->ErrorCode))
    {
      RxCpltCallback(&charRx, 1, 0);
    }
    HAL_UART_Receive_IT(huart, &charRx, 1);
  }
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
