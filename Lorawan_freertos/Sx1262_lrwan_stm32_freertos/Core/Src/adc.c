/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{
  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance                   = ADC1;

  /* --- The following members are correct for STM32F4 --- */
  hadc.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4; /* Example value, adjust as needed */
  hadc.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode          = DISABLE;                       /* For single channel mode */
  hadc.Init.ContinuousConvMode    = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion       = 1;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  /* --- These members do NOT exist on STM32F4 and have been removed --- */
  // hadc.Init.OversamplingMode      = DISABLE;
  // hadc.Init.LowPowerAutoWait      = DISABLE;

  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /* --- The self-calibration function does NOT exist on STM32F4 and has been removed --- */
  // if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel      = ADC_CHANNEL_0; /* Example channel, change as needed */
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC_MspInit 0 */

    /* USER CODE END ADC_MspInit 0 */
    /* ADC clock enable */
    /* Use the instance-specific macro for STM32F4 */
    __HAL_RCC_ADC1_CLK_ENABLE(); 

    /* USER CODE BEGIN ADC_MspInit 1 */
    /* GPIO Port and Pin configuration would go here */
    /* e.g. __HAL_RCC_GPIOA_CLK_ENABLE(); */
    /* GPIO_InitStruct.Pin = GPIO_PIN_0; */
    /* ... etc ... */
    /* USER CODE END ADC_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC_MspDeInit 0 */

    /* USER CODE END ADC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /* USER CODE BEGIN ADC_MspDeInit 1 */
    /* GPIO DeInit would go here */
    /* USER CODE END ADC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
