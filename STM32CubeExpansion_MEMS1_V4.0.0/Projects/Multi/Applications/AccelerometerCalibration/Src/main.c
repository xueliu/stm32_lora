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
  * @mainpage Documentation for MotionAC package of X-CUBE-MEMS1 Software for X-NUCLEO-IKS01A1 and X-NUCLEO-IKS01A2 expansion boards
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * MotionAC software is an add-on for the X-CUBE-MEMS1 software and provides
  * accelerometer calibration.
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
#include "MotionAC_Manager.h"

/** @addtogroup MOTION_AC_Applications
  * @{
  */

/** @addtogroup ACC_CALIB
  * @{
  */

/** @addtogroup Main Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define REPORT_INTERVAL         20 /* ms */

/* Extern variables ----------------------------------------------------------*/
extern int use_LSI;
int RTC_SYNCH_PREDIV;

/* Private variables ---------------------------------------------------------*/
volatile uint32_t DataTxPeriod = REPORT_INTERVAL;
volatile uint32_t timestamp = 0;
volatile uint8_t sensor_read_request = 0;

static RTC_HandleTypeDef RtcHandle;
static SensorAxes_t     ACC_Value;

/* Exported variables --------------------------------------------------------*/
volatile uint8_t DataLoggerActive;
volatile uint32_t Sensors_Enabled = 0;
void *ACCELERO_handle    = NULL;
void *GYRO_handle        = NULL;
void *MAGNETO_handle     = NULL;
void *HUMIDITY_handle    = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle    = NULL;

TMsg Msg;
TIM_HandleTypeDef ACTimHandle;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);

static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_AC_Init(void);
static void RTC_Handler(TMsg *Msg);
static void AC_Data_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_IKS01A2
  *          expansion board to perform accelerometer sensor calibration and send
  *          data from a Nucleo board to a connected PC, using UART, displaying
  *          it on Unicleo-GUI application, developed by STMicroelectronics.
  *          After connection has been established with GUI, the user can visualize
  *          the data and save datalog for offline analysis.
  *          See User Manual for details.
  *
  * @param  None
  * @retval None
  */
int main(void)
{
  char lib_version[35];
  int lib_version_len;

  /* STM32F4xx, STM32L4xx HAL library initialization:
    - Configure the Flash prefetch, instruction and Data caches
    - Configure the Systick to generate an interrupt each 1 msec
    - Set NVIC Group Priority to 4
    - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize GPIOs */
  MX_GPIO_Init();
  MX_CRC_Init();

  /* Initialize (disabled) Sensors */
  Init_Sensors();

  /* Accelerometer calibration API initialization function */
  /* Use default settings */
  MotionAC_manager_init(REPORT_INTERVAL);

  /* OPTIONAL */
  /* Get library version */
  MotionAC_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log */
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Timer for AC algorithm syncronization Initialization */
  MX_TIM_AC_Init();

  while(1)
  {
    if ((UART_ReceivedMSG((TMsg*) &Msg)) && (Msg.Data[0] == DEV_ADDR)) HandleMSG((TMsg*) &Msg);

    if (DataLoggerActive && sensor_read_request)
    {
      sensor_read_request = 0;

      /* Acquire data from enabled sensors and fill Msg stream */
      RTC_Handler(&Msg);
      if(Sensors_Enabled & MAGNETIC_SENSOR   ) Magneto_Sensor_Handler(&Msg);
      if(Sensors_Enabled & GYROSCOPE_SENSOR  ) Gyro_Sensor_Handler(&Msg);
      if(Sensors_Enabled & HUMIDITY_SENSOR   ) Humidity_Sensor_Handler(&Msg);
      if(Sensors_Enabled & TEMPERATURE_SENSOR) Temperature_Sensor_Handler(&Msg);
      if(Sensors_Enabled & PRESSURE_SENSOR   ) Pressure_Sensor_Handler(&Msg);

      /* Accelerometer specific part */
      if(Sensors_Enabled & ACCELEROMETER_SENSOR)
      {
        Accelero_Sensor_Handler(&Msg);
        AC_Data_Handler(&Msg);
      }

      /* Send data stream */
      INIT_STREAMING_HEADER(&Msg);
      Msg.Len = STREAMING_MSG_LENGTH;
      UART_SendMsg(&Msg);
    }
  }
}


/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void Init_Sensors(void)
{
  /* Initialize accelerometer
    - ODR: >= 50Hz
    - FS = +/-4g
    -
  */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_ACCELERO_Set_FS(ACCELERO_handle, FS_MID);
  BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, 50.0f);

  /* Initialize other sensors: not used in Stand_alone Working Mode*/
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle);
  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);
  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
}


/**
  * @brief  GPIO init function.
  * @param  None
  * @retval None
  * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
  */
static void MX_GPIO_Init(void)
{
  /* Initialize LED and push button*/
  BSP_LED_Init(LED2);
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
  * @brief  TIM_AC init function.
  * @param  None
  * @retval None
  * @details This function intialize the Timer used to syncronize the AC algorithm.
  */
static void MX_TIM_AC_Init(void)
{
#define PERIOD_50HZ  ((uint8_t)39)

#if (defined (USE_STM32F4XX_NUCLEO))    /* 84 MHZ CPU clock */
#define PRESCALER_50HZ     ((uint16_t)41999)

#elif (defined (USE_STM32L4XX_NUCLEO))  /* 80 MHZ CPU clock */
#define PRESCALER_50HZ     ((uint16_t)39999)

#else
#error Not supported platform
#endif

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ACTimHandle.Instance = TIM_AC;
  ACTimHandle.Init.Prescaler = PRESCALER_50HZ;
  ACTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  ACTimHandle.Init.Period = PERIOD_50HZ;
  ACTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&ACTimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&ACTimHandle, &sClockSourceConfig);

  HAL_TIM_OC_Init(&ACTimHandle);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&ACTimHandle, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&ACTimHandle, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&ACTimHandle);
}


/**
  * @brief  Handles the time+date getting/sending
  * @param  Msg - time+date part of the stream
  *  @retval None
  */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
  subSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xff);

  Msg->Data[3] = (uint8_t)stimestructure.Hours;
  Msg->Data[4] = (uint8_t)stimestructure.Minutes;
  Msg->Data[5] = (uint8_t)stimestructure.Seconds;
  Msg->Data[6] = subSec;
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
      BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);

      Serialize_s32(&Msg->Data[19], (int32_t)ACC_Value.AXIS_X, 4);
      Serialize_s32(&Msg->Data[23], (int32_t)ACC_Value.AXIS_Y, 4);
      Serialize_s32(&Msg->Data[27], (int32_t)ACC_Value.AXIS_Z, 4);
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
  SensorAxes_t GYR_Value;

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
  * @brief  Accelerometer calibration data sending
  * @param  Msg - Accelerometer calibration part of the stream
  * @retval None
  */
static void AC_Data_Handler(TMsg *Msg)
{
  MAC_input_t data_in;
  MAC_output_t data_out;

  SensorAxes_t ACC_Comp;

  data_in.Acc[0] = ACC_Value.AXIS_X / 1000.0f;
  data_in.Acc[1] = ACC_Value.AXIS_Y / 1000.0f;
  data_in.Acc[2] = ACC_Value.AXIS_Z / 1000.0f;
  data_in.TimeStamp =  timestamp * REPORT_INTERVAL;

  /* Run Accelerometer calibration algorithm */
  MotionAC_manager_update(&data_in);

  /* Get the accelerometer compensation */
  MotionAC_manager_get_params(&data_out);

  /* Do offset & scale factor calibration */
  MotionAC_manager_compensate(&ACC_Value, &ACC_Comp);

  /* Offset coeficients */
  Serialize_s32(&Msg->Data[55], (int32_t) acc_bias_to_mg(data_out.AccBias[0]), 4);
  Serialize_s32(&Msg->Data[59], (int32_t) acc_bias_to_mg(data_out.AccBias[1]), 4);
  Serialize_s32(&Msg->Data[63], (int32_t) acc_bias_to_mg(data_out.AccBias[2]), 4);

  /* Scale factor coeficients */
  FloatToArray(&Msg->Data[67], data_out.SF_Matrix[0][0]);
  FloatToArray(&Msg->Data[71], data_out.SF_Matrix[0][1]);
  FloatToArray(&Msg->Data[75], data_out.SF_Matrix[0][2]);

  FloatToArray(&Msg->Data[79], data_out.SF_Matrix[1][0]);
  FloatToArray(&Msg->Data[83], data_out.SF_Matrix[1][1]);
  FloatToArray(&Msg->Data[87], data_out.SF_Matrix[1][2]);

  FloatToArray(&Msg->Data[91], data_out.SF_Matrix[2][0]);
  FloatToArray(&Msg->Data[95], data_out.SF_Matrix[2][1]);
  FloatToArray(&Msg->Data[99], data_out.SF_Matrix[2][2]);

  /* Calibrated data */
  Serialize_s32(&Msg->Data[103], (int32_t) ACC_Comp.AXIS_X, 4);
  Serialize_s32(&Msg->Data[107], (int32_t) ACC_Comp.AXIS_Y, 4);
  Serialize_s32(&Msg->Data[111], (int32_t) ACC_Comp.AXIS_Z, 4);

  /* Calibration quality */
  Serialize_s32(&Msg->Data[115], (int32_t) data_out.CalQuality, 4);
}


/**
  * @brief  Handles the MAG axes data getting/sending
  * @param  Msg - MAG part of the stream
  * @retval None
  */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  SensorAxes_t MAG_Value;

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
      memcpy(&Msg->Data[15], (void *)&HUMIDITY_Value, sizeof(float));;
    }
  }
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
  * @param  None
  * @retval None
  */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date */
  /* Set Date: Monday March 21st 2016 */
  sdatestructure.Year = 0x16;
  sdatestructure.Month = RTC_MONTH_MARCH;
  sdatestructure.Date = 0x21;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time */
  /* Set Time: 08:10:00 */
  stimestructure.Hours = 0x08;
  stimestructure.Minutes = 0x10;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
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
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
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

}


/**
  * @brief  Period elapsed callback
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM_AC)
  {
    sensor_read_request = 1;
    timestamp++;
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
