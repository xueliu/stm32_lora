/**
  ******************************************************************************
  * @file        main.h
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Header for main.c. module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#ifdef USE_IKS01A2
#include "x_nucleo_iks01a2.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_humidity.h"
#include "x_nucleo_iks01a2_temperature.h"
#elif USE_IKS01A1
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_humidity.h"
#endif

/* Exported defines ----------------------------------------------------------*/
#define LEN_DATABUFFER 15

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t Date[3];
    uint8_t Time[3];
} DataTime_t;

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    AR_STATUS_STANBY                    = 0x00,
    AR_STAUS_ALGO_ON                    = 0x01,
    AR_STATUS_PAUSE_DISPLAY_VALUE       = 0x02
} ar_status_t;

typedef enum
{
  PB_STATUS_NOTEMPTYFLASH               = 0xFF,
  PB_STATUS_FIRST_PUSH                  = 0x01,
  PB_STATUS_FIRST_PUSH_LEDMODE          = 0x02,
  PB_STATUS_SUCC_PUSH                   = 0x00
} pb_status_t;

typedef enum
{
    STANDALONE_M        = 0x00,
    PCGUI_M             = 0x01
} ar_wMode_t;

typedef struct {
    DataTime_t DateTime;
    uint8_t NbOccurr ;
    uint8_t ActivityType;
} DataByteAct_t;

typedef enum
{
    FLAG_ACQ_OFF     = 0x00,
    FLAG_ACQ_ON      = 0x01
} acq_status_t;

/**
 * @brief  Float Axes structure definition
 */
typedef struct {
    float AXIS_X;
    float AXIS_Y;
    float AXIS_Z;
} AxesF_TypeDef;

/* Definition for TIMx clock resources : Timer used for LED dirving */
#define TIM_LEDdrv                            TIM4
#define TIM_LEDdrv_CLK_ENABLE                 __TIM4_CLK_ENABLE
#define TIM_LEDdrv_CLK_DISABLE                __TIM4_CLK_DISABLE
/* Definition for TIMx's NVIC */
#define TIM_LEDdrv_IRQn                       TIM4_IRQn
#define TIM_LEDdrv_IRQHandler                 TIM4_IRQHandler

/* Definition for TIMx clock resources : Timer used for LED dirving */
#define TIM_AR                                TIM3
#define TIM_AR_CLK_ENABLE                     __TIM3_CLK_ENABLE
#define TIM_AR_CLK_DISABLE                    __TIM3_CLK_DISABLE
/* Definition for TIMx's NVIC */
#define TIM_AR_IRQn                           TIM3_IRQn
#define TIM_AR_IRQHandler                     TIM3_IRQHandler

/* Definition for TIMx clock resources */
#define TIMDataLog                            TIM2
#define TIMDataLog_CLK_ENABLE                 __TIM2_CLK_ENABLE
#define TIMDataLog_CLK_DISABLE                __TIM2_CLK_DISABLE

/* Enable sensor masks */
#define PRESSURE_SENSOR                         0x00000001
#define TEMPERATURE_SENSOR                      0x00000002
#define HUMIDITY_SENSOR                         0x00000004
#define ACCELEROMETER_SENSOR                    0x00000010
#define GYROSCOPE_SENSOR                        0x00000020
#define MAGNETIC_SENSOR                         0x00000040

/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

