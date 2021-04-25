/**
  ******************************************************************************
  * @file    FlashMemHandler.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for FLASH Interface functionalities.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASHMEMHANDLER_H
#define FLASHMEMHANDLER_H


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_IF_MIN_WRITE_LEN (8U)  /* Flash programming by 64 bits */
/* Exported functions ------------------------------------------------------- */

struct FlashMemHandlerFct_s
{

  /*!
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval HAL Status.
  */
  HAL_StatusTypeDef(*Init)(void);

  /*!
  * @brief  This function does an erase of n (depends on Length) pages in user flash area
  * @param  pStart: Start of user flash area
  * @param  uLength: number of bytes.
  * @retval HAL status.
  */
  HAL_StatusTypeDef(*Erase_Size)(void *pStart, uint32_t uLength);

  /*!
  * @brief  This function writes a data buffer in flash (data are 64-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  pDestination: Start address for target location
  * @param  pSource: pointer on buffer with data to write
  * @param  uLength: Length of data buffer in byte. It has to be 64-bit aligned.
  * @retval HAL Status.
  */
  HAL_StatusTypeDef(*Write)(void *pDestination, const void *pSource, uint32_t uLength);

  /*!
  * @brief  This function reads flash
  * @param  pDestination: Start address for target location
  * @param  pSource: pointer on buffer with data to write
  * @param  Length: Length in bytes of data buffer
  * @retval HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
  */
  HAL_StatusTypeDef(*Read)(void *pDestination, const void *pSource, uint32_t Length);

} ;

extern const struct  FlashMemHandlerFct_s FlashMemHandlerFct;

#endif  /* FLASHMEMHANDLER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
