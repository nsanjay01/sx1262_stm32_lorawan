/*
 * timer.c - Timer management for LoRaMac-node on STM32F4RE
 * Adapted for STM32CubeMX HAL with RTC and TIM2
 */

 #include "timer.h"
 #include "stm32f4xx_hal.h"
 #include "systime.h"
 
 // External TIM2 and RTC handles from CubeMX-generated main.c
 extern TIM_HandleTypeDef htim2;
 extern RTC_HandleTypeDef hrtc;
 
 // Timer list head
 static TimerEvent_t *TimerListHead = NULL;
 
 // Context for elapsed time reference (in milliseconds)
 static uint32_t TimerContext = 0;
 
 /*!
  * \brief Initializes the timer hardware (TIM2)
  */
 void TimerHwInit(void)
 {
     HAL_TIM_Base_Start_IT(&htim2);
 }
 
 /*!
  * \brief Sets the timer context using RTC
  */
 void RtcSetTimerContext(void)
 {
     SysTime_t sysTime = SysTimeGet();
     TimerContext = (sysTime.Seconds * 1000) + sysTime.SubSeconds;
 }
 
 /*!
  * \brief Gets the current timer context
  */
 uint32_t RtcGetTimerContext(void)
 {
     return TimerContext;
 }
 
 /*!
  * \brief Gets the elapsed time since the last context (in milliseconds)
  */
 uint32_t RtcGetTimerElapsedTime(void)
 {
     SysTime_t sysTime = SysTimeGet();
     uint32_t now = (sysTime.Seconds * 1000) + sysTime.SubSeconds;
     return now - TimerContext;
 }
 
 /*!
  * \brief Gets the current timer value in milliseconds
  */
 uint32_t RtcGetTimerValue(void)
 {
     SysTime_t sysTime = SysTimeGet();
     return (sysTime.Seconds * 1000) + sysTime.SubSeconds;
 }
 
 /*!
  * \brief Timer IRQ handler (called from TIM2_IRQHandler)
  */
 void TimerIrqHandler(void)
 {
     uint32_t elapsedTime = 0;
     TimerEvent_t *current = TimerListHead;
 
     while (current != NULL)
     {
         elapsedTime = RtcGetTimerElapsedTime();
         if (elapsedTime >= current->Timestamp)
         {
             TimerListHead = current->Next;
             current->IsStarted = false;
 
             if (current->Callback != NULL)
             {
                 current->Callback(current->Context);
             }
 
             TimerEvent_t *temp = current;
             current = current->Next;
             temp->Next = NULL;
         }
         else
         {
             break;
         }
     }
 }
 
 /*!
  * \brief Initializes a timer
  */
 void TimerInit(TimerEvent_t *obj, void (*callback)(void *context))
 {
     obj->Timestamp = 0;
     obj->ReloadValue = 0;
     obj->IsStarted = false;
     obj->IsNext2Expire = false;
     obj->Callback = callback;
     obj->Context = NULL;
     obj->Next = NULL;
 }
 
 /*!
  * \brief Sets the timer context
  */
 void TimerSetContext(TimerEvent_t *obj, void *context)
 {
     obj->Context = context;
 }
 
 /*!
  * \brief Starts a timer
  */
 void TimerStart(TimerEvent_t *obj)
 {
     uint32_t elapsedTime = 0;
     __disable_irq();
 
     if ((obj == NULL) || (obj->IsStarted == true))
     {
         __enable_irq();
         return;
     }
 
     obj->Timestamp = obj->ReloadValue;
     obj->IsStarted = true;
 
     if (TimerListHead == NULL)
     {
         TimerListHead = obj;
         RtcSetTimerContext();
     }
     else
     {
         elapsedTime = RtcGetTimerElapsedTime();
         obj->Timestamp += elapsedTime;
 
         TimerEvent_t *current = TimerListHead;
         TimerEvent_t *previous = NULL;
 
         while (current != NULL)
         {
             if (obj->Timestamp < current->Timestamp)
             {
                 if (previous == NULL)
                 {
                     TimerListHead = obj;
                 }
                 else
                 {
                     previous->Next = obj;
                 }
                 obj->Next = current;
                 break;
             }
             previous = current;
             current = current->Next;
         }
 
         if (current == NULL)
         {
             previous->Next = obj;
         }
     }
 
     __enable_irq();
 }
 
 /*!
  * \brief Stops a timer
  */
 void TimerStop(TimerEvent_t *obj)
 {
     __disable_irq();
 
     if ((obj == NULL) || (TimerListHead == NULL))
     {
         __enable_irq();
         return;
     }
 
     TimerEvent_t *previous = TimerListHead;
     TimerEvent_t *current = previous;
 
     while (current != NULL)
     {
         if (current == obj)
         {
             if (current == TimerListHead)
             {
                 TimerListHead = current->Next;
             }
             else
             {
                 previous->Next = current->Next;
             }
             current->IsStarted = false;
             current->Next = NULL;
             break;
         }
         previous = current;
         current = current->Next;
     }
 
     __enable_irq();
 }
 
 /*!
  * \brief Resets a timer
  */
 void TimerReset(TimerEvent_t *obj)
 {
     TimerStop(obj);
     TimerStart(obj);
 }
 
 /*!
  * \brief Sets the timer value
  */
 void TimerSetValue(TimerEvent_t *obj, uint32_t value)
 {
     TimerStop(obj);
     obj->ReloadValue = value;
 }
 
 /*!
  * \brief Computes elapsed time since a past timestamp
  */
 TimerTime_t TimerGetElapsedTime(TimerTime_t past)
 {
     TimerTime_t now = RtcGetTimerValue();
     if (now >= past)
     {
         return now - past;
     }
     return 0; // Prevent underflow (optional, depends on use case)
 }
 
 /*!
  * \brief Gets the current time
  */
 TimerTime_t TimerGetCurrentTime(void)
 {
     return RtcGetTimerValue();
 }