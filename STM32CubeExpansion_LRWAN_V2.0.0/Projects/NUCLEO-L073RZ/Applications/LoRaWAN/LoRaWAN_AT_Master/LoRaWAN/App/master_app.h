/**
  ******************************************************************************
  * @file    master_app.h
  * @author  MCD Application Team
  * @brief   Application of the AT Master
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
#ifndef __MASTER_APP_H__
#define __MASTER_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/



/*!
 * CAYENNE_LPP is myDevices Application server.
 */
/*#define CAYENNE_LPP*/



/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Lora Application
  * @param None
  * @retval None
  */
void MasterApp_Init(void);



#ifdef __cplusplus
}
#endif

#endif /*__MASTER_APP_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
