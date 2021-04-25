/**
  ******************************************************************************
  * @file    app_master.h
  * @author  MCD Application Team
  * @brief   Header of Entry point AT master application
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
#ifndef __APP_MASTER_H__
#define __APP_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

#define DISABLE_IRQ() __disable_irq()
#define ENABLE_IRQ() __enable_irq()

/* Exported Functions Prototypes------------------------------------------------------- */
/**
  * @brief  Init Lora Application
  * @param None
  * @retval None
  */
void MX_Master_Init(void);

/**
  * @brief  Entry Lora Process or scheduling
  * @param None
  * @retval None
  */
void MX_Master_Process(void);

/**
  * @brief  Configures LED GPIO for external modem.
  * @param  Led: Led to be configured.
  * @retval None
  */
void Master_LED_Modem_Init(Led_TypeDef Led);

/**
  * @brief  Turns selected LED Modem On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */
void Master_LED_Modem_On(Led_TypeDef Led);

/**
  * @brief  Turns selected LED Modem Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */
void Master_LED_Modem_Off(Led_TypeDef Led);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__APP_MASTER_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
