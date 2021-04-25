/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_debug.c
  * @author  MCD Application Team
  * @brief   Enables 4 debug pins for internal signals RealTime debugging
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_debug.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void DBG_Init(void)
{
  /* USER CODE BEGIN DBG_Init_1 */

  /* USER CODE END DBG_Init_1 */
#if defined (DEBUGGER_ON) && ( DEBUGGER_ON == 1 )
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  /* Enable the GPIO Clock */
  DGB_LINE1_CLK_ENABLE();
  DGB_LINE2_CLK_ENABLE();
  DGB_LINE3_CLK_ENABLE();
  DGB_LINE4_CLK_ENABLE();

  /* Configure the GPIO pin */
  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull   = GPIO_PULLUP;
  GPIO_InitStruct.Speed  = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin    = DGB_LINE1_PIN;
  HAL_GPIO_Init(DGB_LINE1_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin    = DGB_LINE2_PIN;
  HAL_GPIO_Init(DGB_LINE2_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin    = DGB_LINE3_PIN;
  HAL_GPIO_Init(DGB_LINE3_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin    = DGB_LINE4_PIN;
  HAL_GPIO_Init(DGB_LINE4_PORT, &GPIO_InitStruct);

  /* Reset debug Pins */
  HAL_GPIO_WritePin(DGB_LINE1_PORT, DGB_LINE1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DGB_LINE2_PORT, DGB_LINE2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DGB_LINE3_PORT, DGB_LINE3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DGB_LINE4_PORT, DGB_LINE4_PIN, GPIO_PIN_RESET);

  __HAL_RCC_DBGMCU_CLK_ENABLE();

  HAL_DBGMCU_EnableDBGSleepMode();
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_DBGMCU_EnableDBGStandbyMode();

#elif defined (DEBUGGER_ON) && (DEBUGGER_ON == 0) /* DEBUGGER_OFF */
  /* Put the debugger pin PA13 and P14 in analog for LowPower*/
  /* The 4 debug lines above are simply not set in this case */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode   = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull   = GPIO_NOPULL;
  GPIO_InitStruct.Pin    = (GPIO_PIN_13 | GPIO_PIN_14);
  /* make sure clock is enabled before setting the pins with HAL_GPIO_Init() */
  __HAL_RCC_GPIOA_CLK_ENABLE() ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  __HAL_RCC_GPIOA_CLK_DISABLE() ;

  __HAL_RCC_DBGMCU_CLK_ENABLE();
  HAL_DBGMCU_DisableDBGSleepMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();
  __HAL_RCC_DBGMCU_CLK_DISABLE();

#else
#error "DEBUGGER_ON not defined or out of range <0,1>"
#endif /* DEBUGGER_OFF */
  /* USER CODE BEGIN DBG_Init_Last */

  /* USER CODE END DBG_Init_Last */
}
/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
