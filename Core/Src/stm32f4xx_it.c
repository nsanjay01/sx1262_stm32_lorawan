/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "radio.h"
#include "timer.h"
#include "sys_app.h"
#include "sx1262dvk1das_conf.h"
#include "sx1262dvk1das.h"
// #include "timer.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2; // 
extern RTC_HandleTypeDef hrtc;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  // printf("In NMI_Handler function\n");
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
// void HardFault_Handler(void)
// {
//   /* USER CODE BEGIN HardFault_IRQn 0 */

//   /* USER CODE END HardFault_IRQn 0 */
//   printf("In HardFault_Handler function\n");

//   // Optionally, capture fault info from system fault registers:
//   volatile uint32_t *hfsr = (uint32_t *)0xE000ED2C; // Hard Fault Status Register
//   volatile uint32_t *mmfar = (uint32_t *)0xE000ED34; // MemManage Fault Address Register
//   volatile uint32_t *bfar = (uint32_t *)0xE000ED38; // Bus Fault Address Register
//   volatile uint32_t *cfsr = (uint32_t *)0xE000ED28; // Configurable Fault Status Register

//   printf("HFSR: 0x%08X\n", *hfsr);
//   printf("CFSR: 0x%08X\n", *cfsr);
//   printf("MMFAR: 0x%08X\n", *mmfar);
//   printf("BFAR: 0x%08X\n", *bfar);
//   while (1)
//   {
//     /* USER CODE BEGIN W1_HardFault_IRQn 0 */
//     /* USER CODE END W1_HardFault_IRQn 0 */
//   }
// }



__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile
  (
    "tst lr, #4\n"               // Check which stack pointer to use
    "ite eq\n"
    "mrseq r0, msp\n"            // If 0, use MSP
    "mrsne r0, psp\n"            // Else, use PSP
    "b hard_fault_handler_c\n"   // Branch to C handler
  );
}


void hard_fault_handler_c(uint32_t *stack_address)
{
    // uint32_t r0  = stack_address[0];
    //     uint32_t r1  = stack_address[1];
    //         uint32_t r2  = stack_address[2];
    //             uint32_t r3  = stack_address[3];
    //                 uint32_t r12 = stack_address[4];
    //                     uint32_t lr  = stack_address[5];
    //                         uint32_t pc  = stack_address[6];
    //                             uint32_t psr = stack_address[7];

    //                                 printf("In HardFault_Handler C\n");
    //                                     printf("R0  = 0x%08lX\n", r0);
    //                                         printf("R1  = 0x%08lX\n", r1);
    //                                             printf("R2  = 0x%08lX\n", r2);
    //                                                 printf("R3  = 0x%08lX\n", r3);
    //                                                     printf("R12 = 0x%08lX\n", r12);
    //                                                         printf("LR  = 0x%08lX\n", lr);
    //                                                             printf("PC  = 0x%08lX <- FAULTING INSTRUCTION\n", pc);
    //                                                                 printf("xPSR= 0x%08lX\n", psr);

    //                                                                     // Also print system fault status registers
    //                                                                         printf("HFSR = 0x%08lX\n", SCB->HFSR);
    //                                                                             printf("CFSR = 0x%08lX\n", SCB->CFSR);
    //                                                                                 printf("MMFAR= 0x%08lX\n", SCB->MMFAR);
    //                                                                                     printf("BFAR = 0x%08lX\n", SCB->BFAR);
     
                                                                                            while (1); // halt system
                                                                                            }
                                                                                            


/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  // printf("In MemManage_Handler function\n");
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  // printf("BusFault_Handler function\n");
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  // printf("In UsageFault_Handler function\n");
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  // printf("In SVC_Handler function\n");

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
  // printf("In DebugMon_Handler function\n");
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  // printf("PendSV_Handler function\n");
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  // printf("SysTick_Handler function\n");
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  // printf("SPI2_IRQHandler\n");
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart2.hdmatx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void EXTI17_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hrtc); // 1. Call the generic HAL handler for the RTC alarm
}
/**
  * @brief  Alarm A callback, which is called by the HAL handler above.
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    APP_LOG(TS_ON, VLEVEL_H, "RTC Alarm callback fired!\n");
  UTIL_TIMER_IRQ_Handler(); // 2. Notify the LoRaWAN timer service
}



/* External Interrupt Handler for EXTI Line 10-15 */
void EXTI15_10_IRQHandler(void)
{
    /* Call HAL EXTI handler with the pin that triggered it */
    // HAL_GPIO_EXTI_IRQHandler(DIO1); // B1_Pin is GPIO_PIN_13
    // APP_LOG(TS_ON, VLEVEL_H, "EXTI15_10_IRQHandler triggered by DIO1\n");
         
  // printf("EXTI15_10_IRQHandler function\n");
  HAL_EXTI_IRQHandler(&H_EXTI_4);  // H_EXTI_4 should be configured for line 
}

// void TIM2_IRQHandler(void)
// {
//     HAL_TIM_IRQHandler(&htim2);
//     TimerIrqHandler();
// }

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
