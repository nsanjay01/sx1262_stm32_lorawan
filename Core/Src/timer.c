/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

/******************************************************************************
 * @file    timer.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    02-mars-2018
 * @brief   Timer implementation for STM32.
 *
 *****************************************************************************/

 #include "timer.h"
 #include "stm32f4xx_hal.h"
 #include <stdio.h>
 
 // External TIM2 handle from main.c
 extern TIM_HandleTypeDef htim2;
 
 // Timer list head
 static TimerEvent_t *TimerListHead = NULL;
 
 // Available timers (up to 10, matching ESP32)
 static bool timerInUse[10] = {false};
 
 // TIM2 overflow counter (for long-running applications)
 static volatile uint32_t tim2Overflows = 0;
 
 /**@brief TIM2 interrupt handler
  */
 void TIM2_IRQHandler(void)
 {
     HAL_TIM_IRQHandler(&htim2);
 }
 
 /**@brief TIM2 callback
  */
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
     if (htim->Instance == TIM2)
     {
         tim2Overflows++; // Increment on overflow
         if (TimerListHead != NULL) // Optimize: skip if no timers
         {
             TimerHandleEvents();
         }
     }
 }
 
 /**@brief Initializes the STM32 timer peripheral
  */
 void TimerConfig(void)
 {
     // TIM2 is initialized in MX_TIM2_Init with 1 ms resolution
     HAL_TIM_Base_Start_IT(&htim2);
 }
 
 /**@brief Initializes the timer object
  */
 void TimerInit(TimerEvent_t *obj, void (*callback)(void))
 {
     if (obj == NULL || callback == NULL)
     {
         printf("TIM: Invalid timer object or callback\n");
         return;
     }
 
     // Find an available timer number
     for (int idx = 0; idx < 10; idx++)
     {
         if (!timerInUse[idx])
         {
             timerInUse[idx] = true;
             obj->timerNum = idx;
             obj->oneShot = true;
             obj->Timestamp = 0;
             obj->ReloadValue = 0;
             obj->IsRunning = false;
             obj->Callback = callback;
             obj->Next = NULL;
             return;
         }
     }
     printf("TIM: No more timers available!\n");
 }
 
 /**@brief Starts and adds the timer object to the list of timer events
  */
 void TimerStart(TimerEvent_t *obj)
 {
     if (obj == NULL || obj->IsRunning)
         return;
 
     __disable_irq();
 
     obj->Timestamp = TimerGetCurrentTime() + obj->ReloadValue;
     obj->IsRunning = true;
 
     // Insert into sorted timer list
     TimerEvent_t *cur = TimerListHead;
     TimerEvent_t *prev = NULL;
 
     while (cur != NULL && cur->Timestamp <= obj->Timestamp)
     {
         prev = cur;
         cur = cur->Next;
     }
 
     if (prev == NULL)
     {
         obj->Next = TimerListHead;
         TimerListHead = obj;
     }
     else
     {
         obj->Next = prev->Next;
         prev->Next = obj;
     }
 
     __enable_irq();
 }
 
 /**@brief Stops and removes the timer object from the list of timer events
  */
 void TimerStop(TimerEvent_t *obj)
 {
     if (obj == NULL || !obj->IsRunning)
         return;
 
     __disable_irq();
 
     TimerEvent_t *cur = TimerListHead;
     TimerEvent_t *prev = NULL;
 
     while (cur != NULL && cur != obj)
     {
         prev = cur;
         cur = cur->Next;
     }
 
     if (cur == obj)
     {
         if (prev == NULL)
         {
             TimerListHead = cur->Next;
         }
         else
         {
             prev->Next = cur->Next;
         }
         obj->IsRunning = false;
         obj->Next = NULL;
     }
 
     __enable_irq();
 }
 
 /**@brief Resets the timer object
  */
 void TimerReset(TimerEvent_t *obj)
 {
     TimerStop(obj);
     TimerStart(obj);
 }
 
 /**@brief Set timer new timeout value
  */
 void TimerSetValue(TimerEvent_t *obj, uint32_t value)
 {
     if (obj == NULL)
         return;
 
     bool wasRunning = obj->IsRunning;
     TimerStop(obj);
     obj->ReloadValue = value;
     if (wasRunning)
         TimerStart(obj);
 }
 
 /**@brief Return the time elapsed since a fixed moment in time
  */
 TimerTime_t TimerGetElapsedTime(TimerTime_t savedTime)
 {
     uint32_t now = TimerGetCurrentTime();
     return now - savedTime;
 }
 
 /**@brief Read the current time elapsed since boot
  */
 TimerTime_t TimerGetCurrentTime(void)
 {
     uint32_t counter = __HAL_TIM_GET_COUNTER(&htim2);
     uint32_t ms = (counter % 1000) + (tim2Overflows * 1000); // Handle overflows
     return ms;
 }
 
 /**@brief Processes pending timer events
  */
 void TimerHandleEvents(void)
 {
     TimerEvent_t *cur = TimerListHead;
     TimerEvent_t *prev = NULL;
     uint32_t now = TimerGetCurrentTime();
 
     while (cur != NULL)
     {
         if (cur->IsRunning && now >= cur->Timestamp)
         {
             void (*callback)(void) = cur->Callback;
             if (cur->oneShot)
             {
                 TimerStop(cur);
                 timerInUse[cur->timerNum] = false; // Free timer
             }
             else
             {
                 cur->Timestamp += cur->ReloadValue;
                 // Re-sort if necessary
                 TimerEvent_t *next = cur->Next;
                 if (next != NULL && next->Timestamp < cur->Timestamp)
                 {
                     if (prev == NULL)
                     {
                         TimerListHead = next;
                     }
                     else
                     {
                         prev->Next = next;
                     }
                     TimerStart(cur);
                     cur = TimerListHead;
                     prev = NULL;
                     continue;
                 }
             }
             if (callback != NULL)
             {
                 callback();
             }
         }
         prev = cur;
         cur = cur->Next;
     }
 }