/**
  ******************************************************************************
  * @file        MotionAC_Manager.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       This file includes gyroscope calibration interface functions
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
#include "MotionGC_Manager.h"

/** @addtogroup MOTION_GC_Applications
  * @{
  */

/** @addtogroup GYRO_CALIB
  * @{
  */

/** @addtogroup GYRO_Driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initialises MotionGC algorithm
  * @param  freq  sampling frequency
  * @retval none
  */
void MotionGC_manager_init(float freq)
{
  MotionGC_Initialize(freq);
}

/**
  * @brief  Get the knobs setting of the library
  * @param  knobs pointer to knobs setting structure
  * @retval none
  */
void MotionGC_manager_get_knobs(MGC_knobs_t *knobs)
{
  MotionGC_GetKnobs(knobs);
}

/**
  * @brief  Set the knobs setting of the library
  * @param  knobs pointer to knobs setting structure
  * @retval none
  */
void MotionGC_manager_set_knobs(MGC_knobs_t *knobs)
{
  MotionGC_SetKnobs(knobs);
}

/**
  * @brief  Run gyroscope calibration algorithm and return compensation parameters
  * @param  data_in  pointer to accaleration [g] and angular rate values [dps]
  * @param  gyro_bias  pointer to actual gyroscope offset value in [dps]
  * @param  bias_update  pointer to an integer that is set to 1 if the gyroscope bias was updated, 0 otherwise
  * @retval none
  */
void MotionGC_manager_update(MGC_input_t *data_in, MGC_output_t *gyro_bias, int *bias_update)
{
  MotionGC_Update(data_in, gyro_bias, bias_update);
}

/**
  * @brief  Get the gyroscope compensation parameters
  * @param  bias_mdps  pointer to array of 3 elements containing the offset, one for each axis, in milli degree per second [mdps]
  * @retval none
  */
void MotionGC_manager_get_params(MGC_output_t *gyro_bias)
{
  MotionGC_GetCalParams(gyro_bias);
}

/**
  * @brief  Set the gyroscope compensation parameters
  * @param  bias_mdps  pointer to array of 3 elements containing the offset, one for each axis, in milli degree per second [mdps]
  * @retval none
  */
void MotionGC_manager_set_params(MGC_output_t *gyro_bias)
{
  MotionGC_SetCalParams(gyro_bias);
}

/**
  * @brief  Set new sample frequency
  * @param  freq  new sample frequency in Herz [Hz]
  * @retval none
  */
void MotionGC_manager_set_frequency(float freq)
{
  MotionGC_SetFrequency(freq);
}

/**
  * @brief  Get the library version
  * @param  version  library version string (must be array of 35 char)
  * @retval none
  */
void MotionGC_manager_get_version(char *version, int *length)
{
  *length = MotionGC_GetLibVersion(version);
}

/**
  * @brief  Do offset calibration
  * @param  dataIn  raw gyroscope data
  * @param  dataOut  calibrated data
  * @retval none
  */
void MotionGC_manager_compensate(SensorAxes_t* DataIn, SensorAxes_t* DataOut)
{
  MGC_output_t gyro_bias;

  MotionGC_GetCalParams(&gyro_bias);

  DataOut->AXIS_X = (int32_t) (DataIn->AXIS_X - gyro_bias_to_mdps(gyro_bias.GyroBiasX));
  DataOut->AXIS_Y = (int32_t) (DataIn->AXIS_Y - gyro_bias_to_mdps(gyro_bias.GyroBiasY));
  DataOut->AXIS_Z = (int32_t) (DataIn->AXIS_Z - gyro_bias_to_mdps(gyro_bias.GyroBiasZ));
}

int16_t gyro_bias_to_mdps(float gyro_bias)
{
  if (gyro_bias >= 0)
  {
    return (int16_t) (gyro_bias * 1000.0f + 0.5f);
  }
  else
  {
    return (int16_t) (gyro_bias * 1000.0f - 0.5f);
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
