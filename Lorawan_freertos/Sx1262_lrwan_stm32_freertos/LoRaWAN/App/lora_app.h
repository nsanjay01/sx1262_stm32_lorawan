/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the LRWAN Middleware
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_APP_H__
#define __LORA_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* LoraWAN application configuration (Mw is configured by lorawan_conf.h) */
#define ACTIVE_REGION                               LORAMAC_REGION_US915

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
/*#define CAYENNE_LPP*/

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000

/*!
 * LoRaWAN User application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_USER_APP_PORT                       2

/*!
 * LoRaWAN Switch class application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_SWITCH_CLASS_PORT                   3

/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_CONFIRMED_MSG

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when LORAWAN_ADR_STATE is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE                   DR_0

/*!
 * LoRaWAN default activation type
 */
#define LORAWAN_DEFAULT_ACTIVATION_TYPE             ACTIVATION_TYPE_OTAA

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * Default Unicast ping slots periodicity
 *
 * \remark periodicity is equal to 2^LORAWAN_DEFAULT_PING_SLOT_PERIODICITY seconds
 *         example: 2^3 = 8 seconds. The end-device will open an Rx slot every 8 seconds.
 */
#define LORAWAN_DEFAULT_PING_SLOT_PERIODICITY       4

#define LORAWAN_FORCE_REJOIN_AT_BOOT                false



/*Send*/
#define CFG_APP_LORA_PROCESS_NAME                  "LORA_SEND_PROCESS"
#define CFG_APP_LORA_PROCESS_ATTR_BITS             (0)
#define CFG_APP_LORA_PROCESS_CB_MEM                (0)
#define CFG_APP_LORA_PROCESS_CB_SIZE               (0)
#define CFG_APP_LORA_PROCESS_STACK_MEM             (0)
#define CFG_APP_LORA_PROCESS_PRIORITY              osPriorityNone
#define CFG_APP_LORA_PROCESS_STACK_SIZE            1024

/*Store Context*/
#define CFG_APP_LORA_STORE_CONTEXT_NAME            "LORA_STORE_CONTEXT"
#define CFG_APP_LORA_STORE_CONTEXT_ATTR_BITS       (0)
#define CFG_APP_LORA_STORE_CONTEXT_CB_MEM          (0)
#define CFG_APP_LORA_STORE_CONTEXT_CB_SIZE         (0)
#define CFG_APP_LORA_STORE_CONTEXT_STACK_MEM       (0)
#define CFG_APP_LORA_STORE_CONTEXT_PRIORITY        osPriorityNone
#define CFG_APP_LORA_STORE_CONTEXT_STACK_SIZE      1024

/*Stop Join*/
#define CFG_APP_LORA_STOP_JOIN_NAME                "LORA_STOP_JOIN"
#define CFG_APP_LORA_STOP_JOIN_ATTR_BITS           (0)
#define CFG_APP_LORA_STOP_JOIN_CB_MEM              (0)
#define CFG_APP_LORA_STOP_JOIN_CB_SIZE             (0)
#define CFG_APP_LORA_STOP_JOIN_STACK_MEM           (0)
#define CFG_APP_LORA_STOP_JOIN_PRIORITY            osPriorityNone
#define CFG_APP_LORA_STOP_JOIN_STACK_SIZE          1024

/*LM Handler*/
#define CFG_LM_HANDLER_PROCESS_NAME                "LM_HANDLER_PROCESS"
#define CFG_LM_HANDLER_PROCESS_ATTR_BITS           (0)
#define CFG_LM_HANDLER_PROCESS_CB_MEM              (0)
#define CFG_LM_HANDLER_PROCESS_CB_SIZE             (0)
#define CFG_LM_HANDLER_PROCESS_STACK_MEM           (0)
#define CFG_LM_HANDLER_PROCESS_PRIORITY            osPriorityNone
#define CFG_LM_HANDLER_PROCESS_STACK_SIZE          1024

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Lora Application
  */
void LoRaWAN_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__LORA_APP_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
