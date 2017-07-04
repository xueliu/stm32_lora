/**
  ******************************************************************************
  * @file        DemoSerial.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Handler Serial Protocol
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "com.h"
#include "DemoDatalog.h"
#include "DemoSerial.h"

/** @addtogroup MOTION_FA_Applications
  * @{
  */

/** @addtogroup FITNESS_ACTIVITIES
  * @{
  */

    /** @addtogroup Serial_Protocol Serial_Protocol
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define DATA_TX_LEN  MIN(4, DATABYTE_FA_LEN)

/* Private variables ---------------------------------------------------------*/
volatile uint8_t DataStreamingDest = 2;
#ifdef USE_IKS01A2
uint8_t PresentationString[] = {"MEMS shield demo,11,2.0.0,1.0.0,IKS01A2"};
#elif USE_IKS01A1
uint8_t PresentationString[] = {"MEMS shield demo,11,2.0.0,1.0.0,IKS01A1"};
#endif

/**
  * @brief  Build the reply header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}


/**
  * @brief  Initialize the streaming header
  * @param  Msg the pointer to the header to be initialized
  * @retval None
  */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}


/**
  * @brief  Initialize the streaming message
  * @param  Msg the pointer to the message to be initialized
  * @retval None
  */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint8_t i;
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for(i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }

  Msg->Len = 3;
}


/**
  * @brief  Handle a message
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int HandleMSG(TMsg *Msg)
/*  DestAddr | SouceAddr | CMD | PAYLOAD
 *      1          1        1       N
 */
{
  uint8_t instance;
  uint32_t i;
  uint32_t addf;
  uint16_t countb = 0;

  if (Msg->Len < 2) return 0;
  if (Msg->Data[0] != DEV_ADDR) return 0;
  switch (Msg->Data[2])
  {
    case CMD_Ping:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;

    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      return 1;

    case CMD_Read_PresString:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);

      i = 0;
      while (i < (sizeof(PresentationString) - 1))
      {
        Msg->Data[3 + i] = PresentationString[i];
        i++;
      }

      Msg->Len = 3 + i;
      UART_SendMsg(Msg);
      return 1;

    case CMD_PRESSURE_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      BSP_PRESSURE_Get_Instance(PRESSURE_handle, &instance);

      switch (instance)
      {
#ifdef USE_IKS01A2
        case LPS22HB_P_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;

#elif USE_IKS01A1
        case LPS25HB_P_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;

        case LPS25HB_P_1:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;

        case LPS22HB_P_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;

#endif
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;

    case CMD_HUMIDITY_TEMPERATURE_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], 1, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;

    case CMD_ACCELERO_GYRO_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);

      /* We can check one between accelerometer instance and gyroscope instance */
      BSP_GYRO_Get_Instance(GYRO_handle, &instance);

      switch (instance)
      {
#ifdef USE_IKS01A2
        case LSM6DSL_G_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;

#elif USE_IKS01A1
        case LSM6DS0_G_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;

        case LSM6DS3_G_0:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;

#endif
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;

    case CMD_MAGNETO_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER( Msg );
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );

      switch (instance)
      {
#ifdef USE_IKS01A2
        case LSM303AGR_M_0:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;

#elif USE_IKS01A1
        case LIS3MDL_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;

#endif
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;

    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3) return 0;

      Sensors_Enabled = Deserialize(&Msg->Data[3], 4);

      /* Start enabled sensors */
      if (Sensors_Enabled & PRESSURE_SENSOR     ) BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
      if (Sensors_Enabled & TEMPERATURE_SENSOR  ) BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
      if (Sensors_Enabled & HUMIDITY_SENSOR     ) BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
      if (Sensors_Enabled & ACCELEROMETER_SENSOR) BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
      if (Sensors_Enabled & GYROSCOPE_SENSOR    ) BSP_GYRO_Sensor_Enable(GYRO_handle);
      if (Sensors_Enabled & MAGNETIC_SENSOR     ) BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);

      HAL_TIM_Base_Start_IT(&FA_TimHandle);
      DataLoggerActive = 1;

      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;

    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3) return 0;

      DataLoggerActive = 0;
      HAL_TIM_Base_Stop_IT(&FA_TimHandle);

      /* Disable all sensors */
      BSP_PRESSURE_Sensor_Disable(PRESSURE_handle);
      BSP_TEMPERATURE_Sensor_Disable(TEMPERATURE_handle);
      BSP_HUMIDITY_Sensor_Disable(HUMIDITY_handle);
      BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
      BSP_GYRO_Sensor_Disable(GYRO_handle);
      BSP_MAGNETO_Sensor_Disable(MAGNETO_handle);

      Sensors_Enabled = 0;

      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;

    case CMD_Set_DateTime:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      RTC_DateRegulate(Msg->Data[6], Msg->Data[7], Msg->Data[8], Msg->Data[9]);
      UART_SendMsg(Msg);
      return 1;

    case CMD_UploadFA:
      if (Msg->Len < 3) return 0;

      addf = (Address_FA2F - MOTION_FA_FLASH_ADD);
      Msg->Len = 4;
      Msg->Data[0] = (addf >> 24) & 0xFF;
      Msg->Data[1] = (addf >> 16) & 0xFF;
      Msg->Data[2] = (addf >>  8) & 0xFF;
      Msg->Data[3] = (addf      ) & 0xFF;

      HAL_UART_Transmit(&UartHandle, (uint8_t*)Msg->Data, 4, 5000);
      /* Read buffer from flash */
      for (addf = MOTION_FA_FLASH_ADD; addf < Address_FA2F; addf += (DATA_TX_LEN * sizeof(DataByte_FA_t)))
      {
        Datalog_FillBuffer2BSent(addf, DATA_TX_LEN);
        memcpy(&Msg->Data[0], DataByte_FA, (DATA_TX_LEN * sizeof(DataByte_FA_t)));
        i = MIN ((Address_FA2F - MOTION_FA_FLASH_ADD - countb), (DATA_TX_LEN * sizeof(DataByte_FA_t)));
        countb += DATA_TX_LEN * sizeof(DataByte_FA_t);
        HAL_UART_Transmit(&UartHandle, (uint8_t*)DataByte_FA, i, 5000);
        HAL_Delay(10);
      }

      Datalog_FlashErase();
      BSP_LED_Off(LED2);
      return 1;

    case CMD_Algo_Mode:
      if (Msg->Len < 3) return 0;
      MotionFA_manager_set_activity((MFA_activity_t) Msg->Data[3]);
      MotionFA_manager_reset_counter();
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;

    default:
      return 0;
  }
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
