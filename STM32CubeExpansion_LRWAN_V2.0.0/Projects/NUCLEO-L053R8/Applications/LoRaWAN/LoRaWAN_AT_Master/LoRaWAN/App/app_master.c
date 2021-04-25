/**
  ******************************************************************************
  * @file    app_master.c
  * @author  MCD Application Team
  * @brief   Entry point AT master application
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

/* Includes ------------------------------------------------------------------*/

#include "master_app.h"
#include "stm32_lpm.h"
#include "sys_app.h"
#include "lora_driver.h"
/*#include "stm32_seq.h"*/  /* if using sequencer uncomment */
#include "usart.h"
#include "app_master.h"
#include "sys_conf.h"
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

#define LEDn_MODEM                              1

/* GPIO configuration for I_NUCLEO_LRWAN1 modem's led*/
GPIO_TypeDef *LED_MODEM_PORT[LEDn_MODEM] = {GPIOA};

const uint16_t LED_MODEM_PIN[LEDn_MODEM] = {GPIO_PIN_0};

void MX_Master_Init(void)
{
  /* USER CODE BEGIN MX_LoRaWAN_Init_1 */

  /* USER CODE END MX_LoRaWAN_Init_1 */
  SystemApp_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_2 */

  /* USER CODE END MX_LoRaWAN_Init_2 */


  MasterApp_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_3 */

  /* USER CODE END MX_LoRaWAN_Init_3 */
}

void MX_Master_Process(void)
{

    /*if using sequencer comment the following code*/
    /* run the LoRa Modem state machine*/
    Lora_fsm();
    DISABLE_IRQ();
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway  */
    if ((lora_getDeviceState() == DEVICE_SLEEP) && (HW_UART_Modem_IsNewCharReceived() == RESET))
    {

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 1)
      UTIL_LPM_EnterLowPower();
#elif !defined (LOW_POWER_DISABLE)
#endif
    }
    ENABLE_IRQ();
  /* USER CODE BEGIN MX_LoRaWAN_Process_1 */

  /* USER CODE END MX_LoRaWAN_Process_1 */
  /* if using sequencer uncomment the call to the sequencer*/
  /*UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);*/
  /* USER CODE BEGIN MX_LoRaWAN_Process_2 */

  /* USER CODE END MX_LoRaWAN_Process_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/**
  * @brief  Configures LED GPIO for external modem.
  * @param  Led: Led to be configured.
  * @retval None
  */
void Master_LED_Modem_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct;

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin = LED_MODEM_PIN[Led];
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(LED_MODEM_PORT[Led], &gpioinitstruct);

  /* Reset PIN to switch off the LED */
  HAL_GPIO_WritePin(LED_MODEM_PORT[Led], LED_MODEM_PIN[Led], GPIO_PIN_RESET);
}



/**
  * @brief  DeInit LEDs GPIO for external modem.
  * @param  Led: LED to be de-init.
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  * @retval None
  */
void Master_LED_Modem_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_MODEM_PORT[Led], LED_MODEM_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED_MODEM_PIN[Led];
  HAL_GPIO_DeInit(LED_MODEM_PORT[Led], gpio_init_structure.Pin);
}


/**
  * @brief  Turns selected LED Modem On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */
void Master_LED_Modem_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_MODEM_PORT[Led], LED_MODEM_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED Modem Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */
void Master_LED_Modem_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_MODEM_PORT[Led], LED_MODEM_PIN[Led], GPIO_PIN_RESET);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
