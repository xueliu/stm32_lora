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
#include "main.h"
#include "DemoDatalog.h"

/** @addtogroup MOTION_FA_Applications
  * @{
  */

/** @addtogroup FITNESS_ACTIVITIES
  * @{
  */

/** @addtogroup Demo_Datalog
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTION_FA_FLASH_LESSTHAN2HH  ((uint32_t)0x0805FA5F)

#elif (defined (USE_STM32L4XX_NUCLEO))
#define MOTION_FA_FLASH_LESSTHAN2HH  ((uint32_t)0x080FF25F)

#else
#error Not supported platform
#endif

/* Private types -------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
uint32_t      Address_FA2F = MOTION_FA_FLASH_ADD;
DataByte_FA_t DataByte_FA[DATABYTE_FA_LEN];

/* Private function prototypes -----------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
static void ReadIntFlash(uint32_t nStartAddress, uint32_t *readBuffer, uint32_t nBytesToRead);

#elif (defined (USE_STM32L4XX_NUCLEO))
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
static void ReadIntFlash(uint32_t nStartAddress, uint64_t *readBuffer, uint32_t nBytesToRead);

#else
#error Not supported platform
#endif

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Set current data storage address to the first free address in flash memory
 * @param  None
 * @retval Flash memory state (FLASH_FULL/FLASH_READY)
 */
flash_state_t Datalog_SetAddress(void)
{
  uint32_t flash_sector = MOTION_FA_FLASH_ADD;

  Address_FA2F += Datalog_SearchNextFreeMemoryIndex(&flash_sector);

  if (Address_FA2F > MOTION_FA_FLASH_LESSTHAN2HH)
  {
    return FLASH_FULL;
  }

  else
  {
    return FLASH_READY;
  }
}

/**
 * @brief  Save the Fitness Activities values to memory
 * @param  index_max  Index to last data record in buffer
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char Datalog_SaveFA2Mem(uint8_t index_max)
{

#if (defined (USE_STM32F4XX_NUCLEO))
#define SECTOR7_ADD 0x8060000
  uint32_t Address = SECTOR7_ADD;

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint32_t Address = 0x80FF800;

#else
#error Not supported platform
#endif

  unsigned char Success=1;
  uint8_t idx;
  uint8_t nword;

  HAL_FLASH_Unlock();

#if (defined (USE_STM32F4XX_NUCLEO))
  uint32_t* lpdata = 0;
   /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint64_t* lpdata = 0;
  /* Clear pending flags (if any) */
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

#else
#error Not supported platform
#endif

  for(idx=0; idx < index_max; idx++)
  {

#if (defined (USE_STM32F4XX_NUCLEO))
    lpdata = (uint32_t*)&DataByte_FA[idx];

#elif (defined (USE_STM32L4XX_NUCLEO))
    lpdata = (uint64_t*)&DataByte_FA[idx];

#else
#error Not supported platform
#endif

    if (Address_FA2F < (uint32_t)(Address - index_max * sizeof(DataByte_FA_t)))
    {

#if (defined (USE_STM32F4XX_NUCLEO))
      for (nword = 0; nword < (sizeof(DataByte_FA_t)/4); nword ++)
      {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_FA2F, *(lpdata + nword)) == HAL_OK)
          Address_FA2F += 4;

        else
        {
          /* Error occurred while writing data in Flash memory */
          Error_Handler();
        }
      }

#elif (defined (USE_STM32L4XX_NUCLEO))
      for (nword = 0; nword < (sizeof(DataByte_FA_t)/8); nword ++)
      {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address_FA2F, *(lpdata + nword)) == HAL_OK)
          Address_FA2F += 8;

        else
        {
          /* Error occurred while writing data in Flash memory */
          Error_Handler();
        }
      }

#else
#error Not supported platform
#endif

    }

    else
    {
      Success = 0;
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
  /* Reset Carry Position Values in FLASH */
  unsigned char Success = 1;

  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

#if (defined (USE_STM32F4XX_NUCLEO))
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = MOTION_FA_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;

#elif (defined (USE_STM32L4XX_NUCLEO))
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MOTION_FA_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MOTION_FA_FLASH_ADD);
  EraseInitStruct.NbPages     = 64;

#else
#error Not supported platform
#endif

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

#if (defined (USE_STM32F4XX_NUCLEO))
   /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

#elif (defined (USE_STM32L4XX_NUCLEO))
  /* Clear pending flags (if any) */
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

#else
#error Not supported platform
#endif

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    Error_Handler();
    Success = 0;
  }
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  Address_FA2F = MOTION_FA_FLASH_ADD;

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

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint64_t TmpReadBuffer = 0x00;

#else
#error Not supported platform
#endif

  uint32_t SectorOffset;
  uint32_t Delta = 0;

  ReadIntFlash(*FlashSectorBaseAddress + (MOTION_FA_FLASH_SECTOR_SIZE/2), &TmpReadBuffer, 4);

#if (defined (USE_STM32F4XX_NUCLEO))
    if(TmpReadBuffer != 0xFFFFFFFF)

#elif (defined (USE_STM32L4XX_NUCLEO))
    if(TmpReadBuffer != 0xFFFFFFFFFFFFFFFF)

#else
#error Not supported platform
#endif

  {
    *FlashSectorBaseAddress = *FlashSectorBaseAddress + (MOTION_FA_FLASH_SECTOR_SIZE/2);
    Delta = (MOTION_FA_FLASH_SECTOR_SIZE>>1);
  }

  for(i = 0; i< (MOTION_FA_FLASH_SECTOR_SIZE>>1); i= i + MOTION_FA_FLASH_ITEM_SIZE)
  {
    ReadIntFlash(*FlashSectorBaseAddress + i, &TmpReadBuffer, 4);

#if (defined (USE_STM32F4XX_NUCLEO))
    if(TmpReadBuffer == 0xFFFFFFFF)

#elif (defined (USE_STM32L4XX_NUCLEO))
    if(TmpReadBuffer == 0xFFFFFFFFFFFFFFFF)

#else
#error Not supported platform
#endif

    {
      SectorOffset = i;
      break;
    }
    else
    {
      SectorOffset = MOTION_FA_FLASH_SECTOR_SIZE/2;
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

#elif (defined (USE_STM32L4XX_NUCLEO))
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

#else
#error Not supported platform
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

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint64_t *pdata =NULL;

#else
#error Not supported platform
#endif

  for (i=0; i< lenbuf; i++)
  {

#if (defined (USE_STM32F4XX_NUCLEO))
  pdata= (uint32_t*)&DataByte_FA[i];

#elif (defined (USE_STM32L4XX_NUCLEO))
  pdata= (uint64_t*)&DataByte_FA[i];

#else
#error Not supported platform
#endif

    ReadIntFlash(add_f, pdata, sizeof(DataByte_FA_t));
    add_f += sizeof(DataByte_FA_t);
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
