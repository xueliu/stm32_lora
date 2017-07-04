/**
  ******************************************************************************
  * @file        stm32l4xx_it.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
#include "main.h"
#include "DemoDatalog.h"

/** @addtogroup MOTION_AW_Applications
  * @{
  */

/** @addtogroup ACTIVITY_RECOGNITION_WRIST
  * @{
  */

/** @addtogroup Interrupt_Handlers  Interrupt Handlers
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void TIM_AW_IRQHandler(void);
void TIM_LEDdrv_IRQHandler(void);
void PB_ManagerStatus(void);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles Hard Fault exception.
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SysTick Handler.
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/
/**
* @brief This function handles TIM_AW global interrupt.
* @param  None
* @retval None
*/
void TIM_AW_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&AWTimHandle);
}

/**
* @brief This function handles TIM_LEDdrv global interrupt.
* @param  None
* @retval None
*/
void TIM_LEDdrv_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&LEDdrvTimHandle);
}


/**
* @brief  This function handles External line 10-15 interrupt request.
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{
  PB_ManagerStatus();
  if (WorkingMode == STANDALONE_M)
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}


/**
* @brief  This function provides flash erasing at user button long pressing.
* @param  None
* @retval None
*/
void PB_ManagerStatus(void)
{
  extern aw_status_t motionAW_status;

  if (FirstPush == PB_STATUS_NOTEMPTYFLASH)
  {
    HAL_Delay(3500);
    if(HAL_GPIO_ReadPin(GPIOC, KEY_BUTTON_PIN) == GPIO_PIN_RESET)
    {
      /*Erase Flash*/
      Datalog_FlashErase();
      BSP_LED_Off(LED2);
      motionAW_status = AW_STATUS_STANBY;
    }
    FirstPush = PB_STATUS_FIRST_PUSH;
  }
  if (FirstPush == PB_STATUS_FIRST_PUSH)
  {
    if(HAL_GPIO_ReadPin(GPIOC, KEY_BUTTON_PIN) != GPIO_PIN_RESET)
    {
      WorkingMode = STANDALONE_M;
      FirstPush = PB_STATUS_FIRST_PUSH_LEDMODE;
    }
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
