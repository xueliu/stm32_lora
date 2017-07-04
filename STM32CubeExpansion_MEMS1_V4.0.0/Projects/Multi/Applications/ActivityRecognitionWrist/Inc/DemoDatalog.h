/**
  ******************************************************************************
  * @file        DemoDatalog.h
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Header for DemoDatalog.c
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DEMODATALOG__H
#define __DEMODATALOG__H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTIONAW_FLASH_SECTOR               FLASH_SECTOR_6
#define MOTIONAW_FLASH_SECTOR_SIZE          (SIZE_FLASH_SECTOR_6)
#define SIZE_FLASH_SECTOR_6                    ((uint32_t)0x00020000)
#endif

#if (defined (USE_STM32L4XX_NUCLEO))
#define MOTIONAW_FLASH_SECTOR_SIZE          ((uint32_t)0x00020000)
#endif
#define MOTIONAW_FLASH_ITEM_SIZE            8

/* Exported defines ----------------------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTIONAW_FLASH_ADD                  ((uint32_t)0x08040000)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
#define MOTIONAW_FLASH_ADD                  ((uint32_t)0x080DF800)/*pag 447*/
#endif

#define LEN_DATA_TX 4

/* Imported variables --------------------------------------------------------*/
extern uint8_t          IdxVectordata;
extern uint32_t         AddressAW2F;
extern DataByteAct_t    DataByteAct[];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
unsigned char Datalog_FlashErase(void);
unsigned char Datalog_SaveAct2Mem(void);
uint32_t Datalog_SearchNextFreeMemoryIndex(uint32_t *FlashSectorBaseAddress);
void Datalog_FillBuffer2BSent(uint32_t add_f, uint8_t lenbuf);

#ifdef __cplusplus
}
#endif

#endif /* __DEMODATALOG__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

