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
 * @file    timer.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    02-mars-2018
 * @brief   timer header functions for LoRa.
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

 #ifndef __TIMER_H__
 #define __TIMER_H__
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "stm32f4xx_hal.h"
 
 #define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))
 
 typedef void (*callbackType)(void);
 
 /**@brief Timer object description
  */
 typedef struct TimerEvent_s
 {
     uint8_t timerNum;             /**< Timer number (0-9 for allocation) */
     bool oneShot;                 /**< True if it is a one-shot timer */
     uint32_t Timestamp;           /**< Current timer value (ms since boot) */
     uint32_t ReloadValue;         /**< Timer delay value (ms) */
     bool IsRunning;               /**< Is the timer currently running */
     void (*Callback)(void);       /**< Timer IRQ callback function */
     struct TimerEvent_s *Next;    /**< Pointer to the next Timer object */
 } TimerEvent_t;
 
 /**@brief Timer time variable definition
  */
 #ifndef TimerTime_t
 typedef uint32_t TimerTime_t;
 #endif
 
 /**@brief Initializes the STM32 timer peripheral
  *
  * @details Configures TIM2 with 1 ms resolution for application timers.
  *          Used in conjunction with systime.c for MCU time.
  */
 void TimerConfig(void);
 
 /**@brief Initializes the timer object
  *
  * @remark TimerSetValue must be called before starting the timer.
  *         Initializes timestamp and reload value to 0.
  * @note Supports up to 10 concurrent timers.
  *
  * @param  obj          Structure containing the timer object parameters
  * @param  callback     Function callback called at the end of the timeout
  */
 void TimerInit(TimerEvent_t *obj, void (*callback)(void));
 
 /**@brief Starts and adds the timer object to the list of timer events
  *
  * @param  obj Structure containing the timer object parameters
  */
 void TimerStart(TimerEvent_t *obj);
 
 /**@brief Stops and removes the timer object from the list of timer events
  *
  * @param  obj Structure containing the timer object parameters
  */
 void TimerStop(TimerEvent_t *obj);
 
 /**@brief Resets the timer object
  *
  * @param  obj Structure containing the timer object parameters
  */
 void TimerReset(TimerEvent_t *obj);
 
 /**@brief Set timer new timeout value
  *
  * @param  obj   Structure containing the timer object parameters
  * @param  value New timer timeout value in ms
  */
 void TimerSetValue(TimerEvent_t *obj, uint32_t value);
 
 /**@brief Return the time elapsed since a fixed moment in time
  *
  * @param  savedTime    Fixed moment in time (ms)
  * @retval time         Returns elapsed time in ms
  */
 TimerTime_t TimerGetElapsedTime(TimerTime_t savedTime);
 
 /**@brief Read the current time elapsed since boot
  *
  * @retval current time in ms
  */
 TimerTime_t TimerGetCurrentTime(void);
 
 /**@brief Processes pending timer events
  *
  * @details Called from TimerIrqHandler to handle timer expirations.
  */
 void TimerHandleEvents(void);
 
 /**@brief Timer interrupt handler for LoRaWAN
  *
  * @details Called from TIM2_IRQHandler to process timer interrupts.
  */
 void TimerIrqHandler(void);
 
 #endif // __TIMER_H__