/**
  ******************************************************************************
  * @file        DemoDatalog.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Utilities for DataLog management
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
#include "DemoDatalog.h"

/** @addtogroup MOTION_AR_Applications
  * @{
  */

/** @addtogroup ACTIVITY_RECOGNITION
  * @{
  */

/** @addtogroup Demo_Datalog
  * @{
  */

/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
static void ReadIntFlash(uint32_t nStartAddress, uint32_t *readBuffer, uint32_t nBytesToRead);
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
static void ReadIntFlash(uint32_t nStartAddress, uint64_t *readBuffer, uint32_t nBytesToRead);
#endif

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Save the Activity Code values to memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char Datalog_SaveAct2Mem(void)
{
#if (defined (USE_STM32F4XX_NUCLEO))
  #define SECTOR7_ADD 0x8060000
  uint32_t Address = SECTOR7_ADD;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  uint32_t Address = 0x80FF800;
#endif

  unsigned char Success=1;
  uint8_t idx;

  HAL_FLASH_Unlock();
#if (defined (USE_STM32F4XX_NUCLEO))
  uint32_t* lpdata = 0;
   /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  uint64_t* lpdata = 0;
  /* Clear pending flags (if any) */
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
#endif

  for(idx=0; idx < (IdxVectordata + 1); idx++)
  {
#if (defined (USE_STM32F4XX_NUCLEO))
    lpdata = (uint32_t*)&DataByteAct[idx];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    lpdata = (uint64_t*)&DataByteAct[idx];
#endif
    if (AddressAR2F < (uint32_t)(Address - (IdxVectordata + 1) * sizeof(DataByteAct_t)))
    {
#if (defined (USE_STM32F4XX_NUCLEO))
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressAR2F, *lpdata) == HAL_OK)
        AddressAR2F += 4;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
       if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, AddressAR2F, *lpdata) == HAL_OK)
         AddressAR2F += 8;
#endif
      else
      {
        /* Error occurred while writing data in Flash memory */
        Error_Handler();
      }
#if (defined (USE_STM32F4XX_NUCLEO))
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressAR2F, *(lpdata + 1)) == HAL_OK)
        AddressAR2F += 4;

      else
      {
        /* Error occurred while writing data in Flash memory */
        Error_Handler();
      }
#endif
    }
  }

  HAL_FLASH_Lock();
  return Success;
}

/**
 * @brief  Reset flash (block 6)
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char Datalog_FlashErase(void)
{
  /* Reset Activity Values in FLASH */
  unsigned char Success = 1;

  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

#if (defined (USE_STM32F4XX_NUCLEO))
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = MOTIONAR_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MOTIONAR_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MOTIONAR_FLASH_ADD);
  EraseInitStruct.NbPages     = 64;
#endif

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    Error_Handler();
    Success = 0;
  }
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  AddressAR2F = MOTIONAR_FLASH_ADD;

  return Success;
}



/**
* @brief  Search the next free Flash memory
 * @param  FlashSectorBaseAddress start address of related Flash sector
* @retval SectorOffset value to add to next Flash address to write
*/
uint32_t Datalog_SearchNextFreeMemoryIndex(uint32_t *FlashSectorBaseAddress)
{
  uint32_t i;
#if (defined (USE_STM32F4XX_NUCLEO))
  uint32_t TmpReadBuffer = 0x00;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  uint64_t TmpReadBuffer = 0x00;
#endif
  uint32_t SectorOffset;
  uint32_t Delta = 0;

  ReadIntFlash(*FlashSectorBaseAddress + (MOTIONAR_FLASH_SECTOR_SIZE/2), &TmpReadBuffer, 4);
#if (defined (USE_STM32F4XX_NUCLEO))
    if(TmpReadBuffer != 0xFFFFFFFF)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    if(TmpReadBuffer != 0xFFFFFFFFFFFFFFFF)
#endif
  {
    *FlashSectorBaseAddress = *FlashSectorBaseAddress + (MOTIONAR_FLASH_SECTOR_SIZE/2);
    Delta = (MOTIONAR_FLASH_SECTOR_SIZE>>1);
  }

  for(i = 0; i< (MOTIONAR_FLASH_SECTOR_SIZE>>1); i= i + MOTIONAR_FLASH_ITEM_SIZE)
  {
    ReadIntFlash(*FlashSectorBaseAddress + i, &TmpReadBuffer, 4);
#if (defined (USE_STM32F4XX_NUCLEO))
    if(TmpReadBuffer == 0xFFFFFFFF)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    if(TmpReadBuffer == 0xFFFFFFFFFFFFFFFF)
#endif
    {
      SectorOffset = i;
      break;
    }
    else
    {
      SectorOffset = MOTIONAR_FLASH_SECTOR_SIZE/2;
    }
  }
  return (SectorOffset + Delta);
}

/**
* @brief  Read of FLASH
* @param  nStartAddress FLASH Address where start to read
* @param  nBytesToRead number of bytes to read in  FLASH
* @retval None
*/
#if (defined (USE_STM32F4XX_NUCLEO))
void ReadIntFlash(uint32_t nStartAddress, uint32_t *readBuffer, uint32_t nBytesToRead)
{
  uint32_t addr = nStartAddress;
  while (addr < nStartAddress + nBytesToRead)
  {
    *(readBuffer) = *(__IO uint32_t*)addr;

    addr += 4;
    readBuffer += 1;
  }

}
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
void ReadIntFlash(uint32_t nStartAddress, uint64_t *readBuffer, uint32_t nBytesToRead)
{
  uint32_t addr = nStartAddress;
  while (addr < nStartAddress + nBytesToRead)
  {
    *(readBuffer) = *(__IO uint64_t*)addr;
    addr += 8;

    readBuffer += 1;
  }

}
#endif

/**
* @brief  Read from FLASH and fill the buffer to be sent via USART
* @param  add_f FLASH Address where start to read
* @param  lenbuf length buffer
* @retval None
*/
void Datalog_FillBuffer2BSent(uint32_t add_f, uint8_t lenbuf)
{
  uint32_t i;
#if (defined (USE_STM32F4XX_NUCLEO))
  uint32_t *pdata =NULL;
#endif
  #if (defined (USE_STM32L4XX_NUCLEO))
  uint64_t *pdata =NULL;
#endif

  for (i=0; i< lenbuf; i++)
  {
#if (defined (USE_STM32F4XX_NUCLEO))
  pdata= (uint32_t*)&DataByteAct[i];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  pdata= (uint64_t*)&DataByteAct[i];
#endif
    ReadIntFlash(add_f, pdata,8);
    add_f += 8;
  }
}

#if (defined (USE_STM32L4XX_NUCLEO))
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
#endif /* USE_STM32L4XX_NUCLEO */

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
