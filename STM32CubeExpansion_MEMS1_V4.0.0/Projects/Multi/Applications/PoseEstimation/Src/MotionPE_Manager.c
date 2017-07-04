/**
  ******************************************************************************
  * @file        MotionPE_Manager.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       This file includes pose estimation interface functions
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
#include "MotionPE_Manager.h"

/** @addtogroup MOTION_PE_Applications
  * @{
  */

/** @addtogroup POSE_ESTIMATION
  * @{
  */

/** @addtogroup MotionPE_Manager MotionPE_Manager
  * @{
  */

/* Public functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize the MotionPE engine
  * @param  handle  Handle to accelerometer instance context
  * @retval none
  */
void MotionPE_manager_init(void *handle)
{
  uint8_t instance;
  char acc_orientation[3];

  MotionPE_Initialize();
  BSP_ACCELERO_Get_Instance(handle, &instance);

  switch (instance)
  {

#if (defined (USE_IKS01A1))
    case LSM6DS0_X_0:
      acc_orientation[0] ='e';
      acc_orientation[1] ='n';
      acc_orientation[2] ='u';
      break;

    case LSM6DS3_X_0:
      acc_orientation[0] ='n';
      acc_orientation[1] ='w';
      acc_orientation[2] ='u';
      break;

    default:
      return;

#elif (defined (USE_IKS01A2))
    case LSM6DSL_X_0:
      acc_orientation[0] = 'n';
      acc_orientation[1] = 'w';
      acc_orientation[2] = 'u';
      break;

    case LSM303AGR_X_0:
      acc_orientation[0] = 'n';
      acc_orientation[1] = 'e';
      acc_orientation[2] = 'u';
      break;

    default:
      return;

#else
#error Not supported platform
#endif

  }

  MotionPE_SetOrientation_Acc(acc_orientation);
  return;
}


/**
  * @brief  Run Pose Estimation algorithm
  * @param  data_in  Structure containing input data
  * @param  data_out  Structure containing output data
  * @retval none
  */
void MotionPE_manager_run(MPE_input_t *data_in, MPE_output_t *data_out)
{
  MotionPE_Update(data_in, data_out);
}


/**
  * @brief  Get the library version
  * @param  version  Library version string (must be array of 35 char)
  * @param  length  Library version string length
  * @retval none
  */
void MotionPE_manager_get_version(char *version, int *length)
{
  *length = MotionPE_GetLibVersion(version);
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
