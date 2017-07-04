/**
  ******************************************************************************
  * @file        main.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Main program body
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

/**
  * @mainpage Documentation for MotionAR package of X-CUBE-MEMS1 Software for X-NUCLEO-IKS01A1 and X-NUCLEO-IKS01A2 expansion board
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * MotionAR software is an add-on for the X-CUBE-MEMS1 software and provides
  * real-time activity recognition data.
  * The expansion is built on top of STM32Cube software technology that eases
  * portability across different STM32 microcontrollers.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "com.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "DemoSerial.h"
#include "DemoDatalog.h"
#include "MotionAR_Manager.h"

/** @addtogroup MOTION_AR_Applications
  * @{
  */

/** @addtogroup ACTIVITY_RECOGNITION
  * @{
  */

/** @addtogroup Main Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DRVLED_TIM_PULSE        999
#define DRVLED_TIM_PERIOD       10000

#define TWENTYSEC       333
#define FIVEMIN         15

#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTIONAR_FLASH_LESSTHAN2HH   ((uint32_t)0x0805FA5F)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
#define MOTIONAR_FLASH_LESSTHAN2HH   ((uint32_t)0x080FF25F)
#endif

#define AR_DATATYPE_RESETVAL 0x00

/* Private variables ---------------------------------------------------------*/
uint8_t new_session = 1;
uint8_t activity5min_count = 0;
static uint16_t activity20s_count;
uint32_t flash_sector = MOTIONAR_FLASH_ADD;
static float sensitivity = 0.0f;
ar_status_t motionAR_status;
MAR_output_t current_activity, prev_activity;
RTC_HandleTypeDef RtcHandle;
acq_status_t flag_acq;

/* Exported variables --------------------------------------------------------*/
extern int use_LSI;
int RTC_SYNCH_PREDIV;

uint8_t IdxVectordata = 0;
TIM_HandleTypeDef LEDdrvTimHandle;
TIM_HandleTypeDef ARTimHandle;
TMsg Msg;
pb_status_t FirstPush;
ar_wMode_t WorkingMode;
DataByteAct_t DataByteAct[LEN_DATABUFFER];
DataTime_t DataTime;
MAR_output_t ActivityCode;
AxesF_TypeDef ACC_Value_f;
uint32_t AddressAR2F;

volatile uint32_t Sensors_Enabled = 0;
extern volatile uint8_t DataLoggerActive;

SensorAxesRaw_t ACC_Value_Raw;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;

void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);

static void VarInit(void);
static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_LEDdrv_Init(void);
static void MX_TIM_AR_Init(void);
static void MotionAR_manager_status(ar_status_t motAR_enabled);
static void displayActivity_LED(void);
static void RTC_Handler_ActivityLog(DataTime_t *Vect);
static void AR_Handler(void);
static void ResetVarDataFlash(void);
static void RTC_Handler(TMsg *Msg);
static void ActCode_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief   Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_OLA01A2
 *          expansion board to recognize activity data and send it from a
 *          Nucleo board to a connected PC, using UART, displaying it on Unicleo-GUI.
 *          After connection has been established with Unicleo-GUI application,
 *          the user can visualize activity recognition data and save datalog
 *          for offline analisys.
 *          See User Manual for details.
 *
 * @param  None
 * @retval None
 */
int main(void)
{
  char lib_version[35];
  int lib_version_len;

  uint8_t startLogFlag = 0;

  /* STM32F4xx, STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Variable initialization*/
  VarInit();

  /* Initialize GPIOs */
  MX_GPIO_Init();
  MX_CRC_Init();

  /* Initialize and disable all Sensors */
  Init_Sensors();

  /* Activity Recognition API initialization function */
  MotionAR_manager_init();

  /* OPTIONAL */
  /* Get library version */
  MotionAR_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log*/
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();

  /* Set AddressAR2F to the first free address in flash for data storage*/
  AddressAR2F += Datalog_SearchNextFreeMemoryIndex(&flash_sector);
  if (AddressAR2F > MOTIONAR_FLASH_LESSTHAN2HH)
  {
    BSP_LED_On(LED2);
    FirstPush = PB_STATUS_NOTEMPTYFLASH;
  }

  while(1)
  {
    /* ----- By default: PC GUI mode ----- */
    if (WorkingMode == PCGUI_M)
    {
      if ((UART_ReceivedMSG((TMsg*) &Msg)) && (Msg.Data[0] == DEV_ADDR))
        HandleMSG((TMsg*) &Msg);

      if(DataLoggerActive)
      {
        if (startLogFlag == 0)
        {
          /* Initialize Timer */
          MX_TIM_AR_Init();
          HAL_TIM_Base_Start_IT(&ARTimHandle);
          startLogFlag = 1;
        }
        if (flag_acq == FLAG_ACQ_ON)
        {
          RTC_Handler(&Msg);
          Pressure_Sensor_Handler(&Msg);
          Humidity_Sensor_Handler(&Msg);
          Temperature_Sensor_Handler(&Msg);
          Accelero_Sensor_Handler(&Msg);
          Gyro_Sensor_Handler(&Msg);
          Magneto_Sensor_Handler(&Msg);
          MotionAR_manager_run();
          ActCode_Handler(&Msg);
          INIT_STREAMING_HEADER(&Msg);
          Msg.Len = STREAMING_MSG_LENGTH;
          UART_SendMsg(&Msg);
          flag_acq = FLAG_ACQ_OFF;
        }
      }
      else
      {
        startLogFlag = 0;
      }
    }
    else if (FirstPush == PB_STATUS_FIRST_PUSH_LEDMODE)
    {
      /* ----- Stand-Alone Working Mode ----- */
      FirstPush = PB_STATUS_SUCC_PUSH;

      /* Initialize Timers */
      MX_TIM_LEDdrv_Init();
      MX_TIM_AR_Init();
      HAL_TIM_Base_Start_IT(&ARTimHandle);
      /* Enable Accelerometer */
      BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
    }
  }
}


/**
 * @brief  Variables intitialization function
 * @param  None
 * @retval None
 */
static void VarInit(void)
{
  flag_acq = FLAG_ACQ_OFF;
  AddressAR2F = MOTIONAR_FLASH_ADD;
  FirstPush = PB_STATUS_FIRST_PUSH;
  WorkingMode = PCGUI_M;
  motionAR_status = AR_STATUS_STANBY;

  /* Variable for Activity data memorization in Flash */
  activity20s_count = 0;
  prev_activity = MAR_NOACTIVITY;
  DataByteAct[0].NbOccurr = AR_DATATYPE_RESETVAL;
  DataByteAct[0].ActivityType = AR_DATATYPE_RESETVAL;
}


/**
 * @brief  Initialize and disable all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
  /* Initialize and Configure Accelerometer: FS = +/-4g; ODR >= 16 Hz */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_ACCELERO_Set_FS(ACCELERO_handle, FS_MID);
  BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, 16.0f);
  BSP_ACCELERO_Get_Sensitivity(ACCELERO_handle, &sensitivity);

  /* Initialize other sensors: not used in Stand_alone Working Mode*/
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);
  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle);
  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);
}


/**
 * @brief  GPIO init function.
 * @param  None
 * @retval None
 * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
 */
static void MX_GPIO_Init(void)
{
  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
}


/**
  * @brief  CRC init function.
  * @param  None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}


/**
 * @brief  TIM_LEDdrv init function.
 * @param  None
 * @retval None
 * @details This function intialize the Timer used to drive the LED according to AR algorithm output
 *          The period is set to 5s, i.e. each 5s the LED will blink N-times being:
 *
 *          N = 1 --> STATIONARY;
 *          N = 2 --> WALKING;
 *          N = 3 --> FAST WALKING;
 *          N = 4 --> JOGGING;
 *          N = 5 --> BIKING;
 *          N = 6 --> DRIVING;
 */
static void MX_TIM_LEDdrv_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  LEDdrvTimHandle.Instance = TIM_LEDdrv;
  LEDdrvTimHandle.Init.Prescaler = 41999;
  LEDdrvTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  LEDdrvTimHandle.Init.Period = DRVLED_TIM_PERIOD;
  LEDdrvTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&LEDdrvTimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&LEDdrvTimHandle, &sClockSourceConfig);

  HAL_TIM_OC_Init(&LEDdrvTimHandle);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&LEDdrvTimHandle, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = DRVLED_TIM_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&LEDdrvTimHandle, &sConfigOC, TIM_CHANNEL_1);
}


/**
 * @brief  TIM_AR init function.
 * @param  None
 * @retval None
 * @details This function intialize the Timer used to syncronize the AR algorithm.
 */
static void MX_TIM_AR_Init(void)
{
  #define PERIOD_16HZ     119
#ifdef USE_STM32F4XX_NUCLEO
  #define PRESCALER_16HZ  41999
#else
  #define PRESCALER_16HZ  39999
#endif

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ARTimHandle.Instance = TIM_AR;
  ARTimHandle.Init.Prescaler = PRESCALER_16HZ;
  ARTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  ARTimHandle.Init.Period = PERIOD_16HZ;
  ARTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&ARTimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&ARTimHandle, &sClockSourceConfig);

  HAL_TIM_OC_Init(&ARTimHandle);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ARTimHandle, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 62;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&ARTimHandle, &sConfigOC, TIM_CHANNEL_1);
}


/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg - time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  subSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV+1))& 0xff);

  Msg->Data[3] = (uint8_t)stimestructure.Hours;
  Msg->Data[4] = (uint8_t)stimestructure.Minutes;
  Msg->Data[5] = (uint8_t)stimestructure.Seconds;
  Msg->Data[6] = subSec;
}


/**
 * @brief  Handles the time+date getting before Activity vector to be memorized
 * @param  Vect - time + date
 * @retval None
 */
static void RTC_Handler_ActivityLog(DataTime_t *Vect)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

  Vect->Time[0] = (uint8_t)stimestructure.Hours;
  Vect->Time[1] = (uint8_t)stimestructure.Minutes;
  Vect->Time[2] = (uint8_t)stimestructure.Seconds;
  Vect->Date[0] = (uint8_t)sdatestructureget.Month;
  Vect->Date[1] = (uint8_t)sdatestructureget.Date;
  Vect->Date[2] = (uint8_t)sdatestructureget.Year;
}


/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg - ACC part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;

  if(Sensors_Enabled & ACCELEROMETER_SENSOR)

  {
    if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_ACCELERO_Get_AxesRaw(ACCELERO_handle, &ACC_Value_Raw);
      ACC_Value_f.AXIS_X = (float)(ACC_Value_Raw.AXIS_X * sensitivity);
      ACC_Value_f.AXIS_Y = (float)(ACC_Value_Raw.AXIS_Y * sensitivity);
      ACC_Value_f.AXIS_Z = (float)(ACC_Value_Raw.AXIS_Z * sensitivity);

      Serialize_s32(&Msg->Data[19], (int32_t)ACC_Value_f.AXIS_X, 4);
      Serialize_s32(&Msg->Data[23], (int32_t)ACC_Value_f.AXIS_Y, 4);
      Serialize_s32(&Msg->Data[27], (int32_t)ACC_Value_f.AXIS_Z, 4);
    }
  }
}


/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg - GYR part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;

  if(Sensors_Enabled & GYROSCOPE_SENSOR)
  {
    if(BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
      Serialize_s32(&Msg->Data[31], GYR_Value.AXIS_X, 4);
      Serialize_s32(&Msg->Data[35], GYR_Value.AXIS_Y, 4);
      Serialize_s32(&Msg->Data[39], GYR_Value.AXIS_Z, 4);
    }
  }
}


/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg - MAG part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;

  if(Sensors_Enabled & MAGNETIC_SENSOR)
  {
      if(BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
      {
        BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
        Serialize_s32(&Msg->Data[43], MAG_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[47], MAG_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[51], MAG_Value.AXIS_Z, 4);
      }
  }
}


/**
 * @brief  Activity data sending
 * @param  Msg - ACT part of the stream
 * @retval None
 */
static void ActCode_Handler(TMsg *Msg)
{
  /* Send stream of Gesture Recognition data*/
  Serialize_s32(&Msg->Data[55], (int32_t)ActivityCode, 4);
}


/**
 * @brief  Handles the PRESS sensor data getting/sending.
 * @param  Msg - PRESS part of the stream
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  float PRESSURE_Value;

  if(Sensors_Enabled & PRESSURE_SENSOR)
  {
    if(BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
      memcpy(&Msg->Data[7], (void *)&PRESSURE_Value, sizeof(float));
    }
  }
}


/**
 * @brief  Handles the TEMP axes data getting/sending
 * @param  Msg - TEMP part of the stream
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  float TEMPERATURE_Value;

  if(Sensors_Enabled & TEMPERATURE_SENSOR)
  {
    if(BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
      memcpy(&Msg->Data[11], (void *)&TEMPERATURE_Value, sizeof(float));
    }
  }
}


/**
 * @brief  Handles the HUM axes data getting/sending
 * @param  Msg - HUM part of the stream
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  float HUMIDITY_Value;

  if(Sensors_Enabled & HUMIDITY_SENSOR)
  {
    if(BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
      memcpy(&Msg->Data[15], (void *)&HUMIDITY_Value, sizeof(float));
    }
  }
}


/**
 * @brief  Handles the Activity Recognition
 * @param  Msg
 * @retval None
 */
static void AR_Handler(void)
{
  uint8_t status = 0;

  if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ((WorkingMode == STANDALONE_M))
    {
      /* Read Accelerometer data (float) */
      BSP_ACCELERO_Get_AxesRaw(ACCELERO_handle, &ACC_Value_Raw);
      ACC_Value_f.AXIS_X = (float)(ACC_Value_Raw.AXIS_X * sensitivity);
      ACC_Value_f.AXIS_Y = (float)(ACC_Value_Raw.AXIS_Y * sensitivity);
      ACC_Value_f.AXIS_Z = (float)(ACC_Value_Raw.AXIS_Z * sensitivity);

      activity20s_count++;

      /* Call AR algorithm (LED blinks to warn on status) */
      BSP_LED_On(LED2);
      MotionAR_manager_run();
      BSP_LED_Off(LED2);

      if (activity20s_count == TWENTYSEC)
      {
        /* Activity is stored on RAM each 20 seconds */
        activity20s_count = 0;
        activity5min_count++;
        current_activity = ActivityCode;
        RTC_Handler_ActivityLog(&DataByteAct[IdxVectordata].DateTime);

        if (current_activity == prev_activity)
        {
          if ((new_session > 0) && (IdxVectordata == 0))
          {
            IdxVectordata ++;
            new_session = 0;
          }
          (DataByteAct[IdxVectordata].NbOccurr)++;
          DataByteAct[IdxVectordata].ActivityType = current_activity;

        }
        else
        {
          if (((new_session == 0) && (IdxVectordata != 0))||((new_session != 0) && (IdxVectordata == 0)))
          {
            IdxVectordata ++;
            new_session = 0;
          }
          else if ((new_session == 0) && (IdxVectordata == 0) && (DataByteAct[0].ActivityType != MAR_NOACTIVITY))
          {
            IdxVectordata ++;
          }
          DataByteAct[IdxVectordata].ActivityType = current_activity;
          DataByteAct[IdxVectordata].NbOccurr = 1;
          prev_activity = current_activity;
        }
        if (activity5min_count >= FIVEMIN)
        {
          /* 5 minutes session of activities is stored in FLASH */
          Datalog_SaveAct2Mem();
          ResetVarDataFlash();
        }
      }
    }else //if (WorkingMode == PCGUI_M)
    {
      if(DataLoggerActive)
      {
        flag_acq = FLAG_ACQ_ON;
      }
    }
  } //  if(BSP_IMU_6AXES_isInitialized())
}


/**
 * @brief  Reset variables and index for next data flash. Data may be memorized each 5' or on user button pushing.
 * @param  None
 * @retval None
 */
void ResetVarDataFlash(void)
{
  uint8_t i;

  activity5min_count = 0;
  activity20s_count = 0;
  for (i=0; i< LEN_DATABUFFER; i++)
  {
    DataByteAct[i].NbOccurr = 0;
    DataByteAct[i].ActivityType = MAR_NOACTIVITY;
  }
  if (motionAR_status != AR_STAUS_ALGO_ON )
    new_session = 1;

  IdxVectordata = 0;
}


/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef        RCC_OscInitStruct;

  /*##-1- Configue LSE as RTC clock soucre ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    use_LSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    use_LSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss) {

    RTC_TimeTypeDef stimestructure;

    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.Hours = hh;
    stimestructure.Minutes = mm;
    stimestructure.Seconds = ss;
    stimestructure.SubSeconds = 0;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BIN) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    while(1)
    {}
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  extern ar_status_t motionAR_status;

  if (WorkingMode == STANDALONE_M)
  {
    if(GPIO_Pin == KEY_BUTTON_PIN)
    {
      switch (motionAR_status)
      {
        case AR_STATUS_STANBY:
          BSP_LED_Off(LED2);
          motionAR_status = AR_STAUS_ALGO_ON;
          break;
        case AR_STAUS_ALGO_ON:
          motionAR_status = AR_STATUS_PAUSE_DISPLAY_VALUE;
          break;
        case AR_STATUS_PAUSE_DISPLAY_VALUE:
          motionAR_status = AR_STATUS_STANBY;
          break;
        default:
          break;
      }
      MotionAR_manager_status(motionAR_status);
    }

  }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM_AR)
  {
    AR_Handler();
  }
  else if (htim->Instance==TIM_LEDdrv)
  {
    displayActivity_LED();
  }
}


/**
  * @brief  Manage state machine in stand-alone mode (1-Standby; 2-Algorithm ON; 3-Pause/Display)
  * @param  motAR_enabled: status
  * @retval None
  */
static void MotionAR_manager_status(ar_status_t motAR_enabled)
{
  extern TIM_HandleTypeDef LEDdrvTimHandle, ARTimHandle;

  switch (motAR_enabled)
  {
    case AR_STATUS_STANBY:
      HAL_TIM_OC_Stop(&ARTimHandle, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&LEDdrvTimHandle, TIM_CHANNEL_1);
      break;
    case AR_STAUS_ALGO_ON:
      BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
      HAL_TIM_OC_Start(&ARTimHandle, TIM_CHANNEL_1);
      break;
    case AR_STATUS_PAUSE_DISPLAY_VALUE:
      /* Memorize in flash and reset activity data vector */
      Datalog_SaveAct2Mem();
      ResetVarDataFlash();
      /* Stop Timer triggering AR algo and set timer for Activity visualization by LED*/
      HAL_TIM_OC_Stop(&ARTimHandle, TIM_CHANNEL_1);
      HAL_TIM_Base_Start_IT(&LEDdrvTimHandle);
      HAL_TIM_PWM_Start(&LEDdrvTimHandle, TIM_CHANNEL_1);
      /* Disable Accelerometer */
      BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
      break;
    default:
      break;
  }
}


/**
  * @brief  Set timing intervals for LED blinking in DISPLAY_MODE1
  * @param  None
  * @retval None
  */
static void displayActivity_LED(void)
{
  volatile int8_t act_index;
  volatile int32_t index;

  #define TLONG_LED 0x600000
  #define TSHORT_LED 0x400000

  switch(ActivityCode)
  {
    case MAR_STATIONARY:
    case MAR_WALKING:
    case MAR_FASTWALKING:
      {
        for (act_index = 0; act_index < (uint8_t)ActivityCode; act_index++)
        {
          BSP_LED_On(LED2);
          for (index = 0; index < TLONG_LED; index++);
          BSP_LED_Off(LED2);
          for (index = 0; index < TLONG_LED; index++);
        }
      }
      break;

    case MAR_JOGGING:
    case MAR_BIKING:
    case MAR_DRIVING:
      {
        for (act_index = 0; act_index < (uint8_t)ActivityCode; act_index++)
        {
          BSP_LED_On(LED2);
          for (index = 0; index<TSHORT_LED; index++);
          BSP_LED_Off(LED2);
          for (index = 0; index<TSHORT_LED; index++);
        }
      }
      break;
    default:
      break;
  }
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.0
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}
#endif

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
