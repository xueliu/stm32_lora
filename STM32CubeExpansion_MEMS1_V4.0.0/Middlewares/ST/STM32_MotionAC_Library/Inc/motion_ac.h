/**
  ******************************************************************************
  * @file    motion_ac.h
  * @author  MEMS Application Team
  * @version V2.0.0
  * @date    01-May-2017
  * @brief   Header for motion_ac module
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
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_AC_H_
#define _MOTION_AC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_AC MOTION_AC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup MOTION_AC_Exported_Types MOTION_AC_Exported_Types
  * @{
  */

typedef enum
{
  MAC_CALQSTATUSUNKNOWN = 0,
  MAC_CALQSTATUSPOOR,
  MAC_CALQSTATUSOK,
  MAC_CALQSTATUSGOOD
} MAC_CalQuality_t;

typedef struct
{
  float Acc[3];                 /* Acceleration in X, Y, Z axis in [g] */
  int TimeStamp;                /* Time stamp for accelerometer sensor output in [ms] */
} MAC_input_t;

typedef struct
{
  float AccBias[3];             /* Accelerometer offset value in [g] */
  float SF_Matrix[3][3];        /* Scale factor correction matrix (diagonal matrix) */
  MAC_CalQuality_t CalQuality;  /* Quality factor */
} MAC_output_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_AC_Exported_Functions MOTION_AC_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialize the MotionAC engine
  * @param  sampletime  period in [ms] between the update function call
  * @param  enable  enable or disable the library
  * @retval none
  */
void MotionAC_Initialize(int sampletime, unsigned short int enable);

/**
  * @brief  Run accelerometer calibration algorithm
  * @param  data_in  pointer to accaleration [g] and timestamp values [ms]
  * @retval none
  */
void MotionAC_Update(MAC_input_t *data_in);

/**
  * @brief  Get the accelerometer compensation parameters
  * @param  data_out  pointer to actual accelerometer offset [g], scale factor correction matrix [-] and quality factor
  * @retval none
  */
void MotionAC_GetCalParams(MAC_output_t *data_out);

/**
  * @brief  Get the library version
  * @param  version pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionAC_GetLibVersion(char *version);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_AC_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
