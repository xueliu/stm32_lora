/**
  ******************************************************************************
  * @file    LmhpDataDistribution.c
  * @author  MCD Application Team
  * @brief   To be compliante with new commun lmhandler architecture.
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

#include "LmhpDataDistribution.h"
#include "LmhpClockSync.h"
#include "LmhpRemoteMcastSetup.h"
#include "LmhpFragmentation.h"

#include "LmHandler.h"





/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  Stub to be compliante with new commun lmhandler architecture
  * @param  none
  * @retval return code
  */
LmHandlerErrorStatus_t LmhpDataDistributionInit(void)
{
  /*  if (LmHandlerPackageRegister(PACKAGE_ID_CLOCK_SYNC, NULL) != LORAMAC_HANDLER_SUCCESS)
    {
      return LORAMAC_HANDLER_ERROR;
    }
    else if (LmHandlerPackageRegister(PACKAGE_ID_REMOTE_MCAST_SETUP, NULL) != LORAMAC_HANDLER_SUCCESS)
    {
      return LORAMAC_HANDLER_ERROR;
    }
    else if (LmHandlerPackageRegister(PACKAGE_ID_FRAGMENTATION, &FragmentationParams) != LORAMAC_HANDLER_SUCCESS)
    {
      return LORAMAC_HANDLER_ERROR;
    }
  */
  return LORAMAC_HANDLER_SUCCESS;
}


/**
  * @brief  To be compliante with new commun lmhandler architecture
  * @param  none
  * @retval return code
  */
LmHandlerErrorStatus_t LmhpDataDistributionPackageRegister(uint8_t id, LmhPackage_t **package)
{
  if (package == NULL)
  {
    return LORAMAC_HANDLER_ERROR;
  }
  switch (id)
  {
    case PACKAGE_ID_CLOCK_SYNC:
    {
      *package = LmphClockSyncPackageFactory();
      break;
    }
    case PACKAGE_ID_REMOTE_MCAST_SETUP:
    {
      *package = LmhpRemoteMcastSetupPackageFactory();
      break;
    }
    case PACKAGE_ID_FRAGMENTATION:
    {
      *package = LmhpFragmentationPackageFactory();
      break;
    }
  }

  return LORAMAC_HANDLER_SUCCESS;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
