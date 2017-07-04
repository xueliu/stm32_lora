/**
  ******************************************************************************
  * @file        MotionAW_Manager.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       This file includes activity recognition interface functions
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
#include "MotionAW_Manager.h"

/** @addtogroup MOTION_AW_Applications
  * @{
  */

/** @addtogroup ACTIVITY_RECOGNITION_WRIST
  * @{
  */

/** @addtogroup AW_Driver AW_Driver
  * @{
  */

/* Extern variables ----------------------------------------------------------*/
extern void *GYRO_handle;

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run activity recognition algorithm. This function collects and scale data from accelerometer and calls the Activity Recognition Algo
* @param  None
* @retval None
*/
void MotionAW_manager_run(void)
{
  MAW_input_t  data_in;
  MAW_output_t data_out;

  data_in.AccX = ACC_Value_f.AXIS_X * (float)FROM_MG_TO_G;
  data_in.AccY = ACC_Value_f.AXIS_Y * (float)FROM_MG_TO_G;
  data_in.AccZ = ACC_Value_f.AXIS_Z * (float)FROM_MG_TO_G;

  MotionAW_Update(&data_in, &data_out);
  ActivityCode = data_out.current_activity;
}


/**
* @brief  Initialises MotionAW algorithm
* @param  none
* @retval none
*/
void MotionAW_manager_init(void)
{
  uint8_t instance;
  char acc_orientation[3];

  MotionAW_Initialize();

  BSP_GYRO_Get_Instance( GYRO_handle, &instance);
  switch (instance)
  {
#ifdef USE_IKS01A2
    case LSM6DSL_G_0:
    default:
     acc_orientation[0] ='n';
     acc_orientation[1] ='w';
     acc_orientation[2] ='u';
    break;
#elif USE_IKS01A1
    case LSM6DS3_G_0:
     acc_orientation[0] ='n';
     acc_orientation[1] ='w';
     acc_orientation[2] ='u';
    break;

  case LSM6DS0_G_0:
  default:
     acc_orientation[0] ='e';
     acc_orientation[1] ='n';
     acc_orientation[2] ='u';
    break;
#endif
  }

  MotionAW_SetOrientation_Acc(acc_orientation);
}


/**
  * @brief  Get the library version
  * @param  version  library version string (must be array of 35 char)
  * @retval none
  */
void MotionAW_manager_get_version(char *version, int *length)
{
  *length = MotionAW_GetLibVersion(version);
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
