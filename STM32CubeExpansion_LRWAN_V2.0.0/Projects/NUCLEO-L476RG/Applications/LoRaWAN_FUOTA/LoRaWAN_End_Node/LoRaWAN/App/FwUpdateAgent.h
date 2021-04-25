/**
  ******************************************************************************
  * @file    FwUpdateAgent.h
  * @author  MCD Application Team
  * @brief   Header for FwUpdateAgent.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FWUPDATEAGENT_H__
#define __FWUPDATEAGENT_H__

#ifdef __cplusplus
extern "C" {
#endif


/*structure containing values related to the management of multi-images in Flash*/
typedef struct
{
  uint32_t  MaxSizeInBytes;        /*!< The maximum allowed size for the FwImage in User Flash (in Bytes) */
  uint32_t  DownloadAddr;          /*!< The download address for the FwImage in UserFlash */
  uint32_t  ImageOffsetInBytes;    /*!< Image write starts at this offset */
  uint32_t  ExecutionAddr;         /*!< The execution address for the FwImage in UserFlash */
} FwImageFlashTypeDef;


/**
  * @brief  Run FW Update process.
  * @param  None
  * @retval HAL Status.
  */
void FwUpdateAgentRun(void);

/**
  * @brief  Data file Transfer from Ram to Flash
  * @param  pData Pointer to the buffer.
  * @param  uSize file dimension (Bytes).
  * @retval None
  */
HAL_StatusTypeDef FwUpdateAgentDataTransferFromRamToFlash(uint8_t *pData, uint32_t uFlashDestination, uint32_t uSize);


#ifdef __cplusplus
}
#endif

#endif /*__FWUPDATEAGENT_H__ */
