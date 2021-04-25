/*!
 * \file      main.c
 *
 * \brief     FUOTA interop tests - test 01
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
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
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"

/* include files for Application packages*/
#include "LmhpCompliance.h"
#include "LmhpClockSync.h"
#include "LmhpRemoteMcastSetup.h"
#include "LmhpFragmentation.h"

/* USER CODE BEGIN Includes */
#include "FlashMemHandler.h"

#include "FwUpdateAgent.h"

#include "FragDecoder.h"

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/*!
 * To overcome the SBSFU contraint. The FW file is copied from RAM to FLASH
 *      - SFU_HEADER & FW code will be copied at the rigth place in FLASH
 */
#define OVERCOME_SBSFU_CONSTRAINT       1

/* Private macro -------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * Defines a random delay for application data transmission duty cycle. 5s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        5000

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa endNode send request
  * @param  none
  * @retval none
  */
/*static void SendTxData(void);@@*/

/**
  * @brief  TX timer callback function
  * @param  timer context
  * @retval none
  */
static void OnTxTimerEvent(void *context);


/**
  * @brief  join event callback function
  * @param  params
  * @retval none
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params
  * @retval none
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa endNode has received a frame
  * @param appData
  * @param params
  * @retval None
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
  * @brief callback Will be called each time a Radio IRQ is handled by the MAC layer
  * @param  none
  * @retval none
 */
static void OnMacProcessNotify(void);


/**
  * @brief callback when end node does moving class
  * @param deviceClass
  * @retval None
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief callback when end node does synchronization
  * @param None
  * @retval None
  */
static void OnSysTimeUpdate(void);

/**
  * @brief  Init `data` buffer of `size` starting at address `addr`
  * @param  addr Address start index to erase.
  * @param  size number of bytes.
  * @retval status Init operation status [0: Success, -1 Fail]
  */
static uint8_t FragDecoderErase(uint32_t addr, uint32_t size);

/**
  * @brief callback to store the data fragment received from network
  * @param [in] addr Address start index to write to.
  * @param [in] data Data buffer to be written.
  * @param [in] size Size of data buffer to be written.
  * @retval status [0: OK, -1 KO]
  */
static uint8_t FragDecoderWrite(uint32_t addr, uint8_t *data, uint32_t size);

/**
  * @brief callback to read data fragment which has been stored in memory
  * @param [in] addr Address start index to read from.
  * @param [in] data Data buffer to be read.
  * @param [in] size Size of data buffer to be read.
  * @retval status [0: OK, -1 KO]
  */
static uint8_t FragDecoderRead(uint32_t addr, uint8_t *data, uint32_t size);

/**
  * @brief callback to follow the data fragment downloading
  * @param deviceClass
  * @retval None
  */
static void OnFragProgress(uint16_t fragCounter, uint16_t fragNb, uint8_t fragSize, uint16_t fragNbLost);

/**
  * @brief callback to notify that the compete data block has been received
  * @param deviceClass
  * @retval None
  */
static void OnFragDone(int32_t status, uint32_t size);

static uint32_t Crc32(uint8_t *buffer, uint16_t length);

static LmHandlerErrorStatus_t Lora_App_DeviceTimeReq(void);
/*!
 *  @brief Will be called when an uplink frame has to be proceeesed
 *
 */
static void UplinkProcess(void);

/*!
 *  @brief Will be called to start a  Tx frame sending
 *
 */
static void LoRaWAN_StartTx(void);

/* Private variables ---------------------------------------------------------*/

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
/*static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };@@*/

static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData,
  .OnClassChange =             OnClassChange,
  .OnSysTimeUpdate =           OnSysTimeUpdate
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};



/*!
 * Defines the maximum size for the buffer receiving the fragmentation result.
 *
 * \remark By default FragDecoder.h defines:
 *         \ref FRAG_MAX_NB   313
 *         \ref FRAG_MAX_SIZE 216
 *
 *         In interop test mode will be
 *         \ref FRAG_MAX_NB   21
 *         \ref FRAG_MAX_SIZE 50
 *
 *         FileSize = FRAG_MAX_NB * FRAG_MAX_SIZE
 *
 *         If bigger file size is to be received or is fragmented differently
 *         one must update those parameters.
 *
 * \remark  Memory allocation is done at compile time. Several options have to be foreseen
 *          in order to optimize the memory. Will depend of the Memory management used
 *          Could be Dynamic allocation --> malloc method
 *          Variable Length Array --> VLA method
 *          pseudo dynamic allocation --> memory pool method
 *          Other option :
 *          In place of using the caching memory method we can foreseen to have a direct
 *          flash memory copy. This solution will depend of the SBSFU constraint
 *
 */


#define UNFRAGMENTED_DATA_SIZE                     ( FRAG_MAX_NB * FRAG_MAX_SIZE )


/*
 * Un-fragmented data storage.
 */
static uint8_t UnfragmentedData[UNFRAGMENTED_DATA_SIZE];

static LmhpFragmentationParams_t FragmentationParams =
{
  .DecoderCallbacks =
  {
    .FragDecoderErase = FragDecoderErase,
    .FragDecoderWrite = FragDecoderWrite,
    .FragDecoderRead = FragDecoderRead,
  },

  .OnProgress = OnFragProgress,
  .OnDone = OnFragDone
};


/*
 * Indicates if LoRaMacProcess call is pending.
 *
 * warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;
/*!
 * Indicates if a Tx frame is pending.
 *
 * \warning Set to 1 when OnTxTimerEvent raised
 */
static volatile uint8_t IsTxFramePending = 0;

/*
 * Indicates if the system time has been synchronized
 */
static volatile bool IsClockSynched = false;

/*
 * MC Session Started
 */
static volatile bool IsMcSessionStarted = false;

/*
 * Indicates if the file transfer is done
 */
static volatile bool IsFileTransferDone = false;

/*
 *  Received file computed CRC32
 */
static volatile uint32_t FileRxCrc = 0;



/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;



/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;


/* Exported functions ---------------------------------------------------------*/


void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */

  LED_Init(LED_RED1);
  LED_Init(LED_RED2);

  /* Get LoRa APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW LoraWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Can be removed - Tag used during test session*/
  APP_LOG(TS_OFF, VLEVEL_M, "\n\rTAG to VALIDATE new FW upgrade %d\n\r", __APP_VERSION_RC);

  /* task registration  - uncomment if using sequencer */
  /*UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaWAN_Process), UTIL_SEQ_RFU, LmHandlerProcess);  */
  /*UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_UplinkOnTxTimerEvent), UTIL_SEQ_RFU, UplinkProcess);  */


  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack - compliance protocol package always initialized and activated during Handler Init*/
  LmHandlerInit(&LmHandlerCallbacks);

  /*The LoRa-Alliance Compliance protocol package should always be initialized and activated.*/
  /*LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);*/

  LmHandlerPackageRegister(PACKAGE_ID_CLOCK_SYNC, NULL);

  LmHandlerPackageRegister(PACKAGE_ID_REMOTE_MCAST_SETUP, NULL);

  LmHandlerPackageRegister(PACKAGE_ID_FRAGMENTATION, &FragmentationParams);

  /*LmHandlerPackageRegister(PACKAGE_ID_FIRMWARE_MANAGEMENT, NULL);   */

  LmHandlerConfigure(&LmHandlerParams);

  /* state variable to indicate synchronization done*/
  IsClockSynched = false;

  /* state variable to indicate data block transfer done*/
  IsFileTransferDone = false;

  LmHandlerJoin(ActivationType);

  /* first Tx frame transmission*/
  LoRaWAN_StartTx() ;

}

/* if using sequencer we have just to register UplinkProcess() and LmHandlerProcess()
 * as task
*/
void LoRaWAN_Process(void)
{
  /*Process application uplink*/
  UplinkProcess();

  /*Processes the LoRaMac events*/
  LmHandlerProcess();

  /*If a flag is set at this point, mcu must not enter low power and must loop*/
  DISABLE_IRQ();

  /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
   * and cortex will not enter low power anyway  */
  if ((IsMacProcessPending != 1) && (IsTxFramePending != 1))
  {
#ifndef LOW_POWER_DISABLE
    UTIL_LPM_EnterLowPower();
#endif
  }
  else
  {
    /*reset notification flag*/
    IsMacProcessPending = 0;
  }

  ENABLE_IRQ();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
}


static void LoRaWAN_StartTx(void)
{
  /* send every time timer elapses */
  UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
  UTIL_TIMER_SetPeriod(&TxTimer,  APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND));
  OnTxTimerEvent(NULL);

}

/*!
 * Function executed on TxTimer event to process an uplink frame
 */
static void UplinkProcess(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  uint8_t isPending = 0;
  CRITICAL_SECTION_BEGIN();
  isPending = IsTxFramePending;
  IsTxFramePending = 0;
  CRITICAL_SECTION_END();
  if (isPending == 1)
  {
    if (LmHandlerIsBusy() == true)
    {
      return;
    }

    if (IsMcSessionStarted == false)    /* we are in Class A*/
    {
      if (IsClockSynched == false)    /* we request AppTimeReq to allow FUOTA */
      {
//        status = LmhpClockSyncAppTimeReq();
        APP_PRINTF(" not sync \n\r");
        status = Lora_App_DeviceTimeReq();
        AppDataBuffer[0] = randr(0, 255);
        /* Send random packet */
        LmHandlerAppData_t appData =
        {
          .Buffer = AppDataBuffer,
          .BufferSize = 1,
          .Port = 1
        };
        status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, NULL, true);


      }
      else
      {
        APP_PRINTF("  sync \n\r");
        AppDataBuffer[0] = randr(0, 255);
        /* Send random packet */
        LmHandlerAppData_t appData =
        {
          .Buffer = AppDataBuffer,
          .BufferSize = 1,
          .Port = 1
        };
        status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, NULL, true);
      }
    }
    else  /* Now we are in Class C or in Class B -- FUOTA feature could be activated */
    {
      if (IsFileTransferDone == false)
      {
        /* do nothing up to the transfer done or sent a data user */
      }
      else
      {
        AppDataBuffer[0] = 0x05; // FragDataBlockAuthReq
        AppDataBuffer[1] = FileRxCrc & 0x000000FF;
        AppDataBuffer[2] = (FileRxCrc >> 8) & 0x000000FF;
        AppDataBuffer[3] = (FileRxCrc >> 16) & 0x000000FF;
        AppDataBuffer[4] = (FileRxCrc >> 24) & 0x000000FF;

        /* Send FragAuthReq */
        LmHandlerAppData_t appData =
        {
          .Buffer = AppDataBuffer,
          .BufferSize = 5,
          .Port = 201
        };
        status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, NULL, true);
      }

      if (status == LORAMAC_HANDLER_SUCCESS)
      {
        /* The fragmented transport layer V1.0 doesn't specify any behavior*/
        /* we keep the interop test behavior - CRC32 is returned to the server*/
        APP_PRINTF(" CRC send \n\r");
      }
    }
    /* send application frame - could be put in conditional compilation*/
    /*  Send(NULL);  comment the sending to avoid interference during multicast*/
  }
}




/* Private functions ---------------------------------------------------------*/


static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */

  /* USER CODE END OnRxData_1 */
  if ((appData != NULL) && (params != NULL))
  {
    LED_On(LED_BLUE);
    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
            default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:
        if (appData->BufferSize == 1)
        {
          AppLedStateOn = appData->Buffer[0] & 0x01;
          if (AppLedStateOn == RESET)
          {
            APP_LOG(TS_OFF, VLEVEL_H,   "LED OFF\r\n");

            LED_Off(LED_RED1);
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");

            LED_On(LED_RED1);
          }
        }
        break;
      /* USER CODE BEGIN OnRxData_Switch_case */

      /* USER CODE END OnRxData_Switch_case */
      default:
        /* USER CODE BEGIN OnRxData_Switch_default */

        /* USER CODE END OnRxData_Switch_default */
        break;
    }
  }

  /* USER CODE BEGIN OnRxData_2 */

  /* USER CODE END OnRxData_2 */
}



static void OnTxTimerEvent(void *context)
{
  /* comment if using sequencer and uncomment the call to Seq*/
  IsTxFramePending = 1;
  /* Task is ready to be executed */
  /*UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_UplinkOnTxTimerEvent), CFG_SEQ_Prio_0);*/

  /*Schedule next transmission*/
  UTIL_TIMER_Start(&TxTimer);
}



static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */

  /* USER CODE END OnTxData_1 */
  if ((params != NULL) && (params->IsMcpsConfirm != 0))
  {
    LED_On(LED_RED2) ;


    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
            params->AppData.Port, params->Datarate, params->TxPower);

    APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
    if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
    }
  }

  /* USER CODE BEGIN OnTxData_2 */

  /* USER CODE END OnTxData_2 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */

  /* USER CODE END OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {

      LED_Off(LED_RED1) ;

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }

  /* USER CODE BEGIN OnJoinRequest_2 */

  /* USER CODE END OnJoinRequest_2 */
}


static void OnClassChange(DeviceClass_t deviceClass)
{
  APP_PRINTF("\r\n...... Switch to Class %c done. .......\r\n", "ABC"[deviceClass]);

  switch (deviceClass)
  {
    default:
    case CLASS_A:
    {
      IsMcSessionStarted = false;
      break;
    }
    case CLASS_B:
    {
      /* Inform the server as soon as possible that the end-device has switched to ClassB */
      LmHandlerAppData_t appData =
      {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0
      };
      LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, NULL, true);
      IsMcSessionStarted = true;
      break;
    }
    case CLASS_C:
    {
      IsMcSessionStarted = true;
      /* Switch LED 3 ON*/
#if (INTEROP_TEST_MODE == 1)
      LED_On(LED_BLUE) ;
#endif
      break;
    }
  }
}


static void OnSysTimeUpdate(void)
{
  IsClockSynched = true;
}

static void OnMacProcessNotify(void)
{
  /* if using sequencer comment this line*/
  IsMacProcessPending = 1;

  /* To notify to the sequencer - task is ready to be executed - uncomment if using sequencer*/
  /*UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaWAN_Process), CFG_SEQ_Prio_0); */
}



static void OnFragProgress(uint16_t fragCounter, uint16_t fragNb, uint8_t fragSize, uint16_t fragNbLost)
{
  /* Switch LED 3 OFF for each received downlink */
#if (INTEROP_TEST_MODE == 1)
  LED_Off(LED_BLUE) ;
#endif

  APP_PRINTF("\r\n....... FRAG_DECODER in Progress .......\r\n");
  APP_PRINTF("RECEIVED    : %5d / %5d Fragments\r\n", fragCounter, fragNb);
  APP_PRINTF("              %5d / %5d Bytes\r\n", fragCounter * fragSize, fragNb * fragSize);
  APP_PRINTF("LOST        :       %7d Fragments\r\n\r\n", fragNbLost);
}


static void OnFragDone(int32_t status, uint32_t size)
{

  APP_PRINTF("\r\n....... FRAG_DECODER Finished .......\r\n");
  APP_PRINTF("STATUS      : %d\r\n", status);

#if (INTEROP_TEST_MODE == 1)

  FileRxCrc = Crc32(UnfragmentedData, size);
  IsFileTransferDone = true;
  APP_PRINTF("Size      : %d\r\n", size);
  APP_PRINTF("CRC         : %08X\r\n\r\n", FileRxCrc);

#else /*INTEROP_TEST_MODE == 0*/

#if (OVERCOME_SBSFU_CONSTRAINT == 1)
  /* copy the file from RAM to FLASH & ASK to reboot the device*/
  if (FwUpdateAgentDataTransferFromRamToFlash(UnfragmentedData, REGION_SLOT_1_START, size) == HAL_OK)
  {
    APP_PRINTF("\r\n...... Transfer file RAM to Flash success --> Run  ......\r\n");
    FwUpdateAgentRun();
  }
  else
  {
    APP_PRINTF("\r\n...... Transfer file RAM to Flash Failed  ......\r\n");
  }
#else
  /*Do a request to Run the Secure boot - The file is already in flash*/
  FwUpdateAgentRun();
#endif

#endif


}



#if (INTEROP_TEST_MODE == 1)
static uint32_t Crc32(uint8_t *buffer, uint16_t length)
{
  // The CRC calculation follows CCITT - 0x04C11DB7
  const uint32_t reversedPolynom = 0xEDB88320;

  // CRC initial value
  uint32_t crc = 0xFFFFFFFF;

  if (buffer == NULL)
  {
    return 0;
  }

  for (uint16_t i = 0; i < length; ++i)
  {
    crc ^= (uint32_t)buffer[i];
    for (uint16_t i = 0; i < 8; i++)
    {
      crc = (crc >> 1) ^ (reversedPolynom & ~((crc & 0x01) - 1));
    }
  }

  return ~crc;
}
#endif



static uint8_t FragDecoderErase(uint32_t addr, uint32_t size)
{
  if (size >= UNFRAGMENTED_DATA_SIZE)
  {
    return (uint8_t) - 1; /* Fail */
  }

#if (INTEROP_TEST_MODE == 1)
  for (uint32_t i = 0; i < size; i++)
  {
    UnfragmentedData[addr + i] = 0xFF;
  }
#else /* INTEROP_TEST_MODE == 0 */
  if (FlashMemHandler_Erase_Size((void *)(SFU_IMG_SLOT_DWL_REGION_BEGIN_VALUE + addr), size);
  {
    return -1;
  }
#endif /* INTEROP_TEST_MODE */
  return 0; /* Success */
}

static uint8_t FragDecoderWrite(uint32_t addr, uint8_t *data, uint32_t size)
{
  if (size >= UNFRAGMENTED_DATA_SIZE)
  {
    return (uint8_t) - 1; /* Fail */
  }

#if (INTEROP_TEST_MODE == 1)  /*write fragment in RAM - Caching mode*/
  for (uint32_t i = 0; i < size; i++)
  {
    UnfragmentedData[addr + i] = data[i];
  }
#else
  FlashMemHandler_Write((void *)addr, (uint8_t *)data, size);
#endif

  return 0; // Success
}

static uint8_t FragDecoderRead(uint32_t addr, uint8_t *data, uint32_t size)
{
  if (size >= UNFRAGMENTED_DATA_SIZE)
  {
    return (uint8_t) - 1; /* Fail */
  }

#if (INTEROP_TEST_MODE == 1)   /*Read fragment in RAM - Caching mode*/
  for (uint32_t i = 0; i < size; i++)
  {
    data[i] = UnfragmentedData[addr + i];
  }
#else
  FlashMemHandler_Read((void *)addr, data, size);
#endif

  return 0; // Success
}

static LmHandlerErrorStatus_t Lora_App_DeviceTimeReq(void)
{
  LoRaMacStatus_t status;
  MlmeReq_t mlmeReq;

  mlmeReq.Type = MLME_DEVICE_TIME;

  status = LoRaMacMlmeRequest(&mlmeReq);

  if (status == LORAMAC_STATUS_OK)
  {
    return LORAMAC_HANDLER_SUCCESS;
  }
  else
  {
    return LORAMAC_HANDLER_ERROR;
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
