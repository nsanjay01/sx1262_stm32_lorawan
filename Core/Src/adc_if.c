/**
  ******************************************************************************
  * @file    adc_if.c
  * @author  MCD Application Team
  * @brief   Read status related to the chip (battery level, VREF, chip temperature)
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

/* Includes ------------------------------------------------------------------*/
#include "adc_if.h"
#include "sys_app.h"

/* External variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;

/* Private define ------------------------------------------------------------*/
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FFF7A2A)) // VREFINT calibration value address for STM32F4
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t) 0x1FFF7A2C)) // Temp sensor 30C calibration value address for STM32F4
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FFF7A2E)) // Temp sensor 110C calibration value address for STM32F4
#define VDDA_TEMP_CAL                  ((uint32_t) 3300)
#define VDDA_VREFINT_CAL               ((uint32_t) 3300)

/* Private macro -------------------------------------------------------------*/
#define COMPUTE_TEMPERATURE(TS_ADC_DATA, VDDA_APPLI)                           \
  ((((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / VDDA_TEMP_CAL)                  \
         - (int32_t) *TEMP30_CAL_ADDR)                                          \
      ) * (int32_t)(110 - 30)                                                   \
     ) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + 30                                                                      \
  ))

/* Private function prototypes -----------------------------------------------*/
static uint32_t ADC_ReadChannels(uint32_t channel);

/* Exported functions --------------------------------------------------------*/
void SYS_InitMeasurement(void)
{
  hadc.Instance = ADC1;
}

void SYS_DeInitMeasurement(void)
{
  // De-initialization logic can be added here if needed
}

int16_t SYS_GetTemperatureLevel(void)
{
  uint32_t measuredLevel = 0;
  uint16_t batteryLevelmV = SYS_GetBatteryLevel();

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_TEMPSENSOR);

  // Note: The temperature calculation for F4 might need scaling differently.
  // The computation returns degrees C, not a scaled value.
  return COMPUTE_TEMPERATURE(measuredLevel, batteryLevelmV);
}

uint16_t SYS_GetBatteryLevel(void)
{
  uint16_t batteryLevelmV = 0;
  uint32_t measuredLevel = 0;

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_VREFINT);

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    batteryLevelmV = (((uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL)) / measuredLevel);
  }
  return batteryLevelmV;
}

/* Private Functions Definition -----------------------------------------------*/
static uint32_t ADC_ReadChannels(uint32_t channel)
{
  uint32_t ADCxConvertedValues = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  // The ADC should be initialized once at startup, not in every read cycle.
  // Calling MX_ADC_Init() here is inefficient. Assuming it's called from main.c

  /* Configure Regular Channel for the specific measurement */
  sConfig.Channel = channel;
  sConfig.Rank = 1; /* For STM32F4, rank is a simple integer */
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; /* Use a valid F4 sample time constant */
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* --- The self-calibration function does NOT exist on STM32F4 and has been removed --- */
  // HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Wait for end of conversion */
  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK)
  {
    ADCxConvertedValues = HAL_ADC_GetValue(&hadc);
  }
  
  /** Stop conversion */
  HAL_ADC_Stop(&hadc);

  /* Revert channel configuration to default or disable to avoid interference */
  sConfig.Rank = 0; // Invalidate the rank
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  return ADCxConvertedValues;
}
