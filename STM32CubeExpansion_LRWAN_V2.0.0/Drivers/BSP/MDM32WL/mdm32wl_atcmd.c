/**
  ******************************************************************************
  * @file    mdm32wl_atcmd.c
  * @author  MCD Application Team
  * @brief   at command API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "sys_conf.h"
#include "usart.h"
#include "mdm32wl_atcmd.h"
#include "stm32_tiny_sscanf.h"

#include <stdarg.h>
#include "stm32_tiny_vsnprintf.h"
#include "sys_debug.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

char LoRa_AT_Cmd_Buff[DATA_TX_MAX_BUFF_SIZE];    /* Buffer used for AT cmd transmission */

static uint16_t Offset = 0;   /*write position needed for sendb command*/

static uint8_t aRxBuffer[5];  /* Buffer used for Rx input character */

static char response[DATA_RX_MAX_BUFF_SIZE];
/*has to be the largest of the response*/
/*not only for return code but also for*/
/*return value: example KEY*/

/****************************************************************************/
/*here we have to include a list of AT cmd by the way of #include<file>     */
/*this file will be preprocessed for CmdTab and ATE_RetCode definition      */
/****************************************************************************/

#undef    __ATCMD_MODEM_H__    /*to avoid recursive include*/
#define   AT_CMD_STRING
#define   AT_ERROR_STRING
#undef    AT_CMD_INDEX
#undef    AT_ERROR_INDEX
#include "atcmd_modem.h"   /*to include MDM32WL specific string AT cmd definition*/



/* private functions ------------------------------------------------------- */

static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker);

static HAL_StatusTypeDef at_cmd_send(uint16_t len);

static ATEerror_t at_cmd_receive(void *pdata);

static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp);

static ATEerror_t at_cmd_receive_async_event(void);

static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp, int8_t *Flag);

static ATEerror_t at_cmd_receive_async_event_downlink_data(void *ptr);

/* Exported functions ------------------------------------------------------- */


/******************************************************************************
  * @brief  Configures modem UART interface.
  * @param  None
  * @retval AT_OK in case of success
  * @retval AT_UART_LINK_ERROR in case of failure
  *****************************************************************************/
ATEerror_t Modem_IO_Init(void)
{
  if (HW_UART_Modem_Init(BAUD_RATE) == HAL_OK)
  {
    return AT_OK;
  }
  else
  {
    return AT_UART_LINK_ERROR;
  }
}


/******************************************************************************
  * @brief  Deinitialise modem UART interface.
  * @param  None
  * @retval None
  *****************************************************************************/
void Modem_IO_DeInit(void)
{
  HAL_UART_MspDeInit(&huart2);
}


/******************************************************************************
  * @brief  Handle the AT cmd following their Groupp type
  * @param  at_group AT group [control, set , get)
  *         Cmd AT command
  *         pdata pointer to the IN/OUT buffer
  * @retval module status
  *****************************************************************************/
ATEerror_t  Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata)
{
  ATEerror_t Status = AT_END_ERROR;
  HAL_StatusTypeDef HAL_Status;
  uint16_t Len;

  /*reset At_cmd buffer for each transmission*/
  memset(LoRa_AT_Cmd_Buff, 0x00, sizeof LoRa_AT_Cmd_Buff);

  switch (at_group)
  {
    case AT_CTRL:
    {
      Len = at_cmd_format(Cmd, NULL, CTRL_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (AT_UART_LINK_ERROR);  /*problem on UART transmission*/
      }
      if (Cmd != AT_RESET)
      {
        Status = at_cmd_receive(NULL);
      }
      break;
    }
    case AT_SET:
    {
      Len = at_cmd_format(Cmd, pdata, SET_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (AT_UART_LINK_ERROR);  /*problem on UART transmission*/
      }
      Status = at_cmd_receive(NULL);
      break;
    }
    case AT_GET:
    {
      Len = at_cmd_format(Cmd, pdata, GET_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (AT_UART_LINK_ERROR);  /*problem on UART transmission*/
      }
      Status = at_cmd_receive(pdata);
      break;
    }
    case AT_ASYNC_EVENT:
    {
      if (Cmd == AT_JOIN || Cmd == AT_SEND)
      {
        Status = at_cmd_receive_async_event();
      }
      else
      {
        Status = at_cmd_receive_async_event_downlink_data(pdata);
      }
      break;
    }
    default:
      DBG_PRINTF("unknown group\n\r");
      break;

  } /*end switch (at_group)*/
  return Status;
}


/******************************************************************************
  * @brief  format the cmd in order to be send
  * @param  Cmd AT command
  *         ptr generic pointer to the IN/OUT buffer
  *         Marker to discriminate the Set from the Get
  * @retval length of the formatted frame to be send
  *****************************************************************************/
static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker)
{
  uint16_t len;      /*length of the formatted command*/
  /*Fmt_t Format; */   /*type of format*/
  uint8_t *PtrValue; /*for IN/OUT buffer*/
  uint32_t value;    /*for 32_02X and 32_D*/
  uint8_t value_8;   /*for 8_D*/
  char value_c;      /*for 8_C*/



  switch (Cmd)
  {
    case AT:
    case AT_RESET:
      /*case AT_JOIN:*/     /*not supported anymore in this format for WL - see AN5481 - new format - FORMAT_8_D_PARAM*/
    {
      /*Format = FORMAT_VOID_PARAM;*/
      len = AT_VPRINTF("%s%s\r\n", AT_HEADER, CmdTab[Cmd]);
      break;
    }
      /*case  AT_SEND: */   /* previous send mode (string) no more supported on WL*/
    case  AT_RECV:
    case  AT_VER:
    {
      /*Format = FORMAT_PLAIN_TEXT;*/
      if (Marker == SET_MARKER)
        len = AT_VPRINTF("%s%s%s%d%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER, ((sSendDataString_t *)ptr)->Port,
                         AT_COLON, ((sSendDataString_t *)ptr)->Buffer);
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case  AT_SEND:  /*supported - sendB replaced by send - now there is just one send mode on WL - see AN5481 for the format description*/
    case  AT_RECVB:
    {
      /*Format = FORMAT_BINARY_TEXT; */
      if (Marker == SET_MARKER)
      {
        Offset = AT_VPRINTF("%s%s%s%d%s%d%s", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER, ((sSendDataBinary_t *)ptr)->Port, AT_COLON, ((sSendDataBinary_t *)ptr)->Ack, AT_COLON);
        unsigned i;
        for (i = 0; i < ((sSendDataBinary_t *)ptr)->DataSize; i++)
        {
          Offset += AT_VPRINTF("%02x", ((sSendDataBinary_t *)ptr)->Buffer[i]);
        }
        Offset += AT_VPRINTF("\r\n");
        len = Offset;
        Offset = 0;
      }
      else
      {
        len = AT_VPRINTF("AT%s%s\r\n", CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case AT_APPKEY:
    case AT_NWKSKEY:
    case AT_APPSKEY:
    {
      /*Format = FORMAT_16_02X_PARAM;*/
      PtrValue = (uint8_t *) ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("AT%s%s%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                         CmdTab[Cmd], AT_SET_MARKER, PtrValue[0], PtrValue[1], PtrValue[2], PtrValue[3],
                         PtrValue[4], PtrValue[5], PtrValue[6], PtrValue[7],
                         PtrValue[8], PtrValue[9], PtrValue[10], PtrValue[11],
                         PtrValue[12], PtrValue[13], PtrValue[14], PtrValue[15]);
      }
      else
      {
        len = AT_VPRINTF("AT%s%s\r\n", CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case AT_DADDR:
    case AT_NWKID:
    {
      /*Format = FORMAT_32_02X_PARAM;*/
      value =  *(uint32_t *)ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%02x:%02x:%02x:%02x\r\n", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER,
                         (unsigned)((unsigned char *)(&value))[3],
                         (unsigned)((unsigned char *)(&value))[2],
                         (unsigned)((unsigned char *)(&value))[1],
                         (unsigned)((unsigned char *)(&value))[0]);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case AT_APPEUI:
    case AT_DEUI:
    {
      /*Format = FORMAT_8_02X_PARAM;*/
      PtrValue = (uint8_t *)ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n", AT_HEADER,
                         CmdTab[Cmd], AT_SET_MARKER, PtrValue[0], PtrValue[1], PtrValue[2],
                         PtrValue[3], PtrValue[4], PtrValue[5], PtrValue[6], PtrValue[7]);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case  AT_RX2FQ:
    case  AT_RX1DL:    /* could be unsigned format - to be analyzed*/
    case  AT_RX2DL:
    case  AT_JN1DL:
    case  AT_JN2DL:
    case  AT_FCU:
    case  AT_FCD:
    {
      /*Format = FORMAT_32_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value =  *(uint32_t *)ptr;
        len = AT_VPRINTF("%s%s%s%u\r\n", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER, value);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case  AT_JOIN:     /*new format supported - see AN5481*/
    case  AT_DR:
    case  AT_RX2DR:
    case  AT_TXP:
    case  AT_NJM:
    case  AT_PNM:
    case  AT_DCS:
    case  AT_ADR:
    case  AT_CFM:
    case  AT_CFS:
    case  AT_BAT:
    case  AT_RSSI:
    case  AT_SNR:
    case  AT_NJS:
    case  AT_VEL:       /*new added cmd - see AN5481*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t *)ptr;
        len = AT_VPRINTF("%s%s%s%d\r\n", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER, value_8);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    case  AT_CLASS:
    {
      /*Format = FORMAT_8_C_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_c =  *(char *)ptr;
        len = AT_VPRINTF("%s%s%s%c\r\n", AT_HEADER, CmdTab[Cmd], AT_SET_MARKER, value_c);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], AT_GET_MARKER);
      }
      break;
    }
    default:
      len = AT_VPRINTF("%s%s%s\r\n", AT_HEADER, CmdTab[Cmd], Marker);
      DBG_PRINTF("format not yet supported \n\r");
      break;
  } /*end switch(cmd)*/
  return len;
}


/******************************************************************************
  * @brief This function sends an AT cmd to the slave device
  * @param len: length of the AT cmd to be sent
  * @retval HAL return code
  ******************************************************************************/
static HAL_StatusTypeDef at_cmd_send(uint16_t len)
{
  HAL_StatusTypeDef RetCode;

  /*transmit the command from master to slave*/
  RetCode = HAL_UART_Transmit(&huart2, (uint8_t *)LoRa_AT_Cmd_Buff, len, 5000);
  return (RetCode);
}



/******************************************************************************
  * @brief This function receives response from the slave device
  * @param pdata: pointer to the value returned by the slave
  * @retval return code coming from slave
  ******************************************************************************/
static ATEerror_t at_cmd_receive(void *pdata)
{
  uint8_t  ResponseComplete = 0;
  int8_t i = 0;
  int8_t charnumber = 0;
  ATEerror_t RetCode;
  uint8_t NoReturnCode = 1;  /*to discriminate the Get return code from return value*/

  /*cleanup the response buffer*/
  memset(response, 0x00, 16);

  /*UART peripheral in reception process for response returned by slave*/
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) != HAL_OK)
  {
    while (1);
  }

  while (!ResponseComplete)
  {
    while (HW_UART_Modem_IsNewCharReceived() == RESET);

    /*process the response*/
    response[i] = HW_UART_Modem_GetNewChar();

    /*wait up to carriage return OR the line feed marker*/
    if (/*(response[i] =='\r') || */(response[i] == '\n'))
    {
      if (pdata == NULL) /*return code following a SET cmd or simple AT cmd*/
      {
        if (i > 1) /*return code following a SET cmd or simple AT cmd- we skip the first <cr><ln>*/
        {
          i = 0;
          ResponseComplete = 1;
          RetCode = at_cmd_responseAnalysing(response);
          break;
        }
      }
      else    /* returned value following a GET cmd */
      {
        if (i != 0 && NoReturnCode)
        {
          /*first statement to get back the return value*/
          response[i] = '\0';
          strcpy(pdata, response);
          memset(response, 0x00, 16);
          i = -1; /*to compensate the next index iteration and restart in [0]*/
          NoReturnCode = 0;  /*return code for the Get cmd*/
        }
        else
        {
          if (i > 1)
          {
            /*second statement to get back the return code*/
            i = 0;
            ResponseComplete = 1;   /*when value + return code have been trapped*/
            RetCode = at_cmd_responseAnalysing(response);
            memset(response, 0x00, 16);
            break;
          }
        }
      }
    }
    else
    {
      if (i == (DATA_RX_MAX_BUFF_SIZE - 1)) /* frame overflow */
      {
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) ;
    charnumber++;
  } /*end while(!Responsablecomplete)*/
  huart2.gState = HAL_UART_STATE_READY;
  huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
  return (RetCode);                             /*version of HAL .. there was not Rx field state*/
}


/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param none
  * @retval return code coming from slave
  ******************************************************************************/
static ATEerror_t at_cmd_receive_async_event(void)
{
  uint8_t  ResponseComplete = 0;
  int8_t i = 0;
  int8_t charnumber = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  uint8_t NoReturnCode = 1;  /*too discriminate the Get return code from return value*/

  /*cleanup the response buffer*/
  memset(response, 0x00, 16);

  /*UART peripheral in reception process for response returned by slave*/
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) != HAL_OK)
  {
    while (1);
  }

  while (!ResponseComplete)
  {

    while (HW_UART_Modem_IsNewCharReceived() == RESET);

    /*process the response*/
    response[i] = HW_UART_Modem_GetNewChar();

    /*wait up to carriage return OR the line feed marker*/
    if (/*(response[i] =='\r') || */(response[i] == '\n'))
    {
      if (i != 0 && NoReturnCode)     /*trap the asynchronous event*/
      {
        /*first statement to get back the return value*/
        response[i] = '\0';
        ptrChr = strchr(&response[0], '+');      /*to point on "+" event statement*/
        RetCode = at_cmd_AsyncEventAnalysing(ptrChr, NULL);
        memset(response, 0x00, 16);
        i = -1;            /*to compensate the next index iteration and restart in [0]*/
        NoReturnCode = 0;  /*return code for the Get cmd*/
        break;
      }
      else
      {
        if (i > 1)
        {
          /*second statement to get back the return code*/
          i = 0;
          ResponseComplete = 1;   /*when value + return code have been trapped*/
          RetCode = at_cmd_responseAnalysing(response);
          memset(response, 0x00, 16);
          break;
        }
      }
    }
    else
    {
      if (i == (DATA_RX_MAX_BUFF_SIZE - 1)) /* frame overflow */
      {
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) ;
    charnumber++;
  }
  huart2.gState = HAL_UART_STATE_READY;
  huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
  return (RetCode);                             /*version of HAL .. there was not Rx field state*/
}


/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param pdata from
  * @retval return code coming from slave
  ******************************************************************************/
static ATEerror_t at_cmd_receive_async_event_downlink_data(void *pdata)
{

  int8_t i = 0;
  int8_t charnumber = 0;
  char *ptrChr;
  ATEerror_t RetCode;
  uint8_t NoReturnCode = 1;  /*too discriminate the Get return code from return value*/
  int8_t DlinkData_Complete = (0x0U);

  /*cleanup the response buffer*/
  memset(response, 0x00, 16);

  /*UART peripheral in reception process for response returned by slave*/
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) != HAL_OK)
  {
    while (1);
  }


  while (!DlinkData_Complete)   /*received sequence not complete*/
  {

    while (HW_UART_Modem_IsNewCharReceived() == RESET);

    /*process the response*/
    response[i] = HW_UART_Modem_GetNewChar();

    /*wait up to carriage return OR the line feed marker*/
    if ((response[i] == '\r') || (response[i] == '\n'))
    {
      if (i != 0 && NoReturnCode)     /*trap the asynchronous events associated to network downlink data*/
      {
        /*sequence of events to be trapped: +EVT:<port>:<size>:<payload>*/
        response[i] = '\0';
        ptrChr = strchr(&response[0], '+');  /*Here when we go out from low power mode the prefix is*/
        /* '\r' only. We do not skip the '\0''\r' - USI behavior ...*/
        RetCode = at_cmd_AsyncEventAnalysing(ptrChr, &DlinkData_Complete);
        if (RetCode == AT_OK)
        {
          ptrChr = strchr(&response[1], ':');      /*to skip the "+EVT:"*/
          strcpy(pdata, ptrChr + 1);
        }

        memset(response, 0x00, 16);
        i = -1;            /*to compensate the next index iteration and restart in [0]*/
      }
    }
    else
    {
      if (i == (DATA_RX_MAX_BUFF_SIZE - 1)) /* frame overflow */
      {
        i = 0;
        return (AT_TEST_PARAM_OVERFLOW);
      }
    }
    i++;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1) ;
    charnumber++;
  }
  huart2.gState = HAL_UART_STATE_READY;
  huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
  return (RetCode);                             /*version of HAL .. there was not Rx field state*/
}


/******************************************************************************
  * @brief This function does analysis of the response received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type
  ******************************************************************************/
static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp)
{
  ATEerror_t status;
  int i;

  status = AT_END_ERROR;

  for (i = 0; i < AT_END_ERROR; i++)
  {
    if (strncmp(ReturnResp, ATE_RetCode[i].RetCodeStr, (ATE_RetCode[i].SizeRetCodeStr - 1)) == 0)
    {
      /* command has been found found*/
      status = ATE_RetCode[i].RetCode;
      return (status);
    }
  }
  return (status);
}


/******************************************************************************
  * @brief This function does analysis of the asynchronous event received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type
  * @Note : list of asynchronous event
  * @Note : +EVT:JOINED / +EVT:JOIN FAILED / +EVT:SEND_CONFIRMED /
  * @Note : +EVT:<port>:<size>:<payload>  / +EVT:<Rx_wind>,<DR>,<RSSI>,<SNR>
  ******************************************************************************/
static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp, int8_t *Flag)
{
  ATEerror_t status;

  status = AT_END_ERROR;

  if (strncmp(ReturnResp, "+EVT:JOINED\r", sizeof("+EVT:JOINED\r") - 1) == 0)
  {
    /* event has been identified*/
    status = AT_OK;
    return (status);
  }

  if (strncmp(ReturnResp, "+EVT:JOIN FAILED\r", sizeof("+EVT:JOIN FAILED\r") - 1) == 0)
  {
    /* event has been identified*/
    status = AT_NO_NET_JOINED;
    return (status);
  }

  if (strncmp(ReturnResp, "+EVT:SEND_CONFIRMED\r", sizeof("+EVT:SEND_CONFIRMED\r") - 1) == 0)
  {
    /* event has been identified*/
    status = AT_OK;
    return (status);
  }

  status = AT_OK ;  /* we are in the default case where we received data from network*/
  *Flag = 1;         /* to notify to the caller the received data*/

  return (status);
}

/******************************************************************************
  * @brief format the AT frame to be sent to the modem (slave)
  * @param pointer to the format string
  * @retval len of the string to be sent
  ******************************************************************************/
uint16_t at_cmd_vprintf(const char *format, ...)
{
  va_list args;
  uint16_t len;

  va_start(args, format);

  len = tiny_vsnprintf_like(LoRa_AT_Cmd_Buff + Offset, sizeof(LoRa_AT_Cmd_Buff) - Offset, format, args);

  va_end(args);

  return len;
}
/* Private functions ---------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
