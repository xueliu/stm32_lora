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

/** @addtogroup MOTION_FX_Applications
  * @{
  */

/** @addtogroup FUSION_DATALOG
  * @{
  */

/** @addtogroup Demo_Datalog
  * @{
  */

/* Private defines -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#if (defined (USE_STM32L4XX_NUCLEO))
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Save the Magnetometer calibration values to memory
 * @param  None
 * @retval None
 */
void SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data)
{
  uint32_t Address = MOTION_FX_FLASH_ADD;

  /* Reset Before The data in Memory */
  ResetCalibrationInMemory();

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

#if (defined (USE_STM32F4XX_NUCLEO))
  uint32_t word;

  for (int nword = 0; nword < dataSize; nword++)
  {
    word = *(data + nword);

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, word) == HAL_OK)
    {
      Address += 4;
    }
    else
    {
      Error_Handler();
    }
  }

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint64_t lword, hword;
  for (int nword = 0; nword < dataSize; nword += 2)
  {
    lword = *(data + nword);

    if ((nword + 1) < dataSize)
    {
      hword = (uint64_t)(*(data + nword + 1)) << 32;
    }
    else
    {
      hword = 0;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, hword | lword) == HAL_OK)
    {
      Address += 8;
    }
    else
    {
      Error_Handler();
    }
  }

#else
#error Not supported platform
#endif

  HAL_FLASH_Lock();
}


/**
 * @brief  Check if there are valid calibration values in memory and read them
 * @param  None
 * @retval None
 */
void RecallCalibrationFromMemory(uint16_t dataSize, uint32_t *data)
{
  uint32_t Address = MOTION_FX_FLASH_ADD;

#if (defined (USE_STM32F4XX_NUCLEO))
  while (Address < (MOTION_FX_FLASH_ADD + dataSize * 4))
  {
    *(data) = *(__IO uint32_t*)Address;
    data += 1;
    Address += 4;
  }

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint32_t *dataMem = data;
  uint64_t dword;

  while (Address < (MOTION_FX_FLASH_ADD + dataSize * 4))
  {
    dword = (*(__IO uint64_t*)Address);
    *(data) = dword & 0x00000000FFFFFFFF;
    data += 1;

    if (data < (dataMem + dataSize))
    {
      *(data) = (uint32_t) ((dword & 0xFFFFFFFF00000000) >> 32);
      data += 1;
    }

    Address += 8;
  }

#else
#error Not supported platform
#endif
}


/**
 * @brief  Reset the Magnetometer calibration values in memory
 * @param  None
 * @retval None
 */
void ResetCalibrationInMemory(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

#if (defined (USE_STM32F4XX_NUCLEO))
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = MOTION_FX_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;

#elif (defined (USE_STM32L4XX_NUCLEO))
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MOTION_FX_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MOTION_FX_FLASH_ADD);
  EraseInitStruct.NbPages     = 1;

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
  }

  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();
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
