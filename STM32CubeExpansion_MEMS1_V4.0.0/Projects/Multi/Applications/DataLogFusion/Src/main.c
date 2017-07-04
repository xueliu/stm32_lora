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
  * @mainpage Documentation for MotionFX package of X-CUBE-MEMS1 Software for
  * X-NUCLEO-IKS01A1 and X-NUCLEO-IKS01A2 expansion board
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * MotionFX software is an add-on for the X-CUBE-MEMS1 software and provides
  * real-time sensor fusion data.
  * The expansion is built on top of STM32Cube software technology that eases
  * portability across different STM32 microcontrollers.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "com.h"
#include "DemoSerial.h"
#include "MotionFX_Manager.h"

/** @addtogroup MOTION_FX_Applications
  * @{
  */

/** @addtogroup DATALOG_FUSION
  * @{
  */

/** @addtogroup Main Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SAMPLE_FREQ                     ((uint8_t)100)  /* [Hz] */
#define SAMPLE_PERIOD                   ((uint8_t)10)   /* [ms] */
#define MOTIONFX_ENGINE_DELTATIME       0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f

/* Public variables ----------------------------------------------------------*/
uint8_t DataLoggerActive = 0;
uint32_t Sensors_Enabled = 0;
uint32_t mag_time_stamp = 0;
uint8_t mag_cal_status = 0;

extern uint8_t SF_6X_Enabled;

int use_LSI = 0;

SensorAxes_t ACC_Value;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;
SensorAxes_t MAG_Offset;

void *ACCELERO_handle    = NULL;
void *GYRO_handle        = NULL;
void *MAGNETO_handle     = NULL;
void *HUMIDITY_handle    = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle    = NULL;

TMsg Msg;
TIM_HandleTypeDef FX_TimHandle;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile static uint8_t sensor_read_request     = 0;
volatile static uint8_t magcal_request          = 0;

static RTC_HandleTypeDef RtcHandle;

static int RTC_SYNCH_PREDIV;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_FX_Init(void);
static void RTC_Handler(TMsg *Msg);
static void FX_Data_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);

/* Public functions ----------------------------------------------------------*/
/**
 * @brief   Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_OLA01A2
 *          expansion board to get postion data and send it from a Nucleo board 
 *          to a connected PC, using UART, displaying it on Unicleo-GUI.
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

  /* Initialize CRC */
  MX_CRC_Init();

  /* Initialize (disabled) Sensors */
  Init_Sensors();

  /* Sensor Fusion API initialization function */
  MotionFX_manager_init(GYRO_handle);

  /* OPTIONAL */
  /* Get library version */
  MotionFX_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log */
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Timer for FX algorithm synchronization Initialization */
  MX_TIM_FX_Init();

  /* LED Blink */
  BSP_LED_On(LED2);
  HAL_Delay(500);
  BSP_LED_Off(LED2);

  /* Enable magnetometer calibration */
  MotionFX_manager_MagCal_start(SAMPLE_PERIOD);

  /* Test if calibration data are available */
  MFX_MagCal_output_t mag_cal_test;
  MotionFX_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available lood HI coeficients */
  if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
  {
    MAG_Offset.AXIS_X = (int32_t) (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
    MAG_Offset.AXIS_Y = (int32_t) (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
    MAG_Offset.AXIS_Z = (int32_t) (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);

    mag_cal_status = 1;
    BSP_LED_On(LED2);
  }

  while(1)
  {
    if ((UART_ReceivedMSG((TMsg*)&Msg)) && (Msg.Data[0] == DEV_ADDR))
        HandleMSG((TMsg*)&Msg);

    if (magcal_request)
    {
      magcal_request = 0;

      /* Reset magnetometer calibration value*/
      mag_cal_status = 0;
      MAG_Offset.AXIS_X = 0;
      MAG_Offset.AXIS_Y = 0;
      MAG_Offset.AXIS_Z = 0;

#if (defined (MOTION_FX_STORE_CALIB_FLASH))
      /* Reset values in memory */
      ResetCalibrationInMemory();
#endif

      /* Enable magnetometer calibration */
      MotionFX_manager_MagCal_start(SAMPLE_PERIOD);

      /* Switch off the LED */
      BSP_LED_Off(LED2);
    }

    if (sensor_read_request)
    {
      sensor_read_request = 0;

      /* Acquire data from enabled sensors and fill Msg stream */
      RTC_Handler(&Msg);
      Accelero_Sensor_Handler(&Msg);
      Gyro_Sensor_Handler(&Msg);
      Magneto_Sensor_Handler(&Msg);
      Humidity_Sensor_Handler(&Msg);
      Temperature_Sensor_Handler(&Msg);
      Pressure_Sensor_Handler(&Msg);

      /* Sensor Fusion specific part */
      FX_Data_Handler(&Msg);

      /* Send data stream */
      INIT_STREAMING_HEADER(&Msg);
      Msg.Len = STREAMING_MSG_LENGTH;
      UART_SendMsg(&Msg);
    }
  }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void Init_Sensors(void)
{
  /* Initialize and Configure all sensors */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);
  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle);
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
  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize push button */
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
  * @brief  TIM_FX init function.
  * @param  None
  * @retval None
  * @details This function intialize the Timer used to syncronize the FX algorithm
  */
static void MX_TIM_FX_Init(void)
{
#define PERIOD_100HZ  ((uint8_t)20)

#if (defined (USE_STM32F4XX_NUCLEO))    /* 84 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)41999)

#elif (defined (USE_STM32L4XX_NUCLEO))  /* 80 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)39999)

#else
#error Not supported platform
#endif

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  FX_TimHandle.Instance = TIM_FX;
  FX_TimHandle.Init.Prescaler = PRESCALER_100HZ;
  FX_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  FX_TimHandle.Init.Period = PERIOD_100HZ;
  FX_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&FX_TimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&FX_TimHandle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&FX_TimHandle, &sMasterConfig);
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
  subSec = ((((((int)RTC_SYNCH_PREDIV) - ((int)stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xFF);

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

  if (Sensors_Enabled & ACCELEROMETER_SENSOR)
  {
    if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
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

  if (Sensors_Enabled & GYROSCOPE_SENSOR)
  {
    if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
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
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;

  if (Sensors_Enabled & MAGNETIC_SENSOR)
  {
    if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);

      if (mag_cal_status == 0)
      {
        mag_data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
        mag_data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
        mag_data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;
        mag_data_in.time_stamp = mag_time_stamp;
        mag_time_stamp += SAMPLE_PERIOD;

        MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

        if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
        {
          mag_cal_status = 1;

          MAG_Offset.AXIS_X = (int32_t) (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
          MAG_Offset.AXIS_Y = (int32_t) (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
          MAG_Offset.AXIS_Z = (int32_t) (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);

          /* Disable magnetometer calibration */
          MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);

          /* Switch on the LED */
          BSP_LED_On(LED2);
        }
      }

      MAG_Value.AXIS_X = (int32_t) (MAG_Value.AXIS_X - MAG_Offset.AXIS_X);
      MAG_Value.AXIS_Y = (int32_t) (MAG_Value.AXIS_Y - MAG_Offset.AXIS_Y);
      MAG_Value.AXIS_Z = (int32_t) (MAG_Value.AXIS_Z - MAG_Offset.AXIS_Z);

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

  if (Sensors_Enabled & PRESSURE_SENSOR)
  {
    if (BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
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

  if (Sensors_Enabled & TEMPERATURE_SENSOR)
  {
    if (BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
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

  if (Sensors_Enabled & HUMIDITY_SENSOR)
  {
    if (BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
      memcpy(&Msg->Data[15], (void *)&HUMIDITY_Value, sizeof(float));
    }
  }
}


/**
  * @brief  Sensor Fusion data handler
  * @param  Msg - Sensor Fusion data part of the stream
  * @retval None
  */
static void FX_Data_Handler(TMsg *Msg)
{
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;

  if ((Sensors_Enabled & ACCELEROMETER_SENSOR) && (Sensors_Enabled & GYROSCOPE_SENSOR) && (Sensors_Enabled & MAGNETIC_SENSOR))
  {
    data_in.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
    data_in.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
    data_in.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS;

    data_in.acc[0] = ACC_Value.AXIS_X * FROM_MG_TO_G;
    data_in.acc[1] = ACC_Value.AXIS_Y * FROM_MG_TO_G;
    data_in.acc[2] = ACC_Value.AXIS_Z * FROM_MG_TO_G;

    data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
    data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
    data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;

    /* Run Sensor Fusion algorithm */
    MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);

    if (SF_6X_Enabled == 1)
    {
       memcpy(&Msg->Data[55], (void *)pdata_out->quaternion_6X, 4 * sizeof(float));
       memcpy(&Msg->Data[71], (void *)pdata_out->rotation_6X, 3 * sizeof(float));
       memcpy(&Msg->Data[83], (void *)pdata_out->gravity_6X, 3 * sizeof(float));
       memcpy(&Msg->Data[95], (void *)pdata_out->linear_acceleration_6X, 3 * sizeof(float));
       memcpy(&Msg->Data[107], (void *)&(pdata_out->heading_6X), sizeof(float));
    }
    else
    {
       memcpy(&Msg->Data[55], (void *)pdata_out->quaternion_9X, 4 * sizeof(float));
       memcpy(&Msg->Data[71], (void *)pdata_out->rotation_9X, 3 * sizeof(float));
       memcpy(&Msg->Data[83], (void *)pdata_out->gravity_9X, 3 * sizeof(float));
       memcpy(&Msg->Data[95], (void *)pdata_out->linear_acceleration_9X, 3 * sizeof(float));
       memcpy(&Msg->Data[107], (void *)&(pdata_out->heading_9X), sizeof(float));
    }
  }
}


/**
  * @brief  Configures the RTC peripheral
  * @param  None
  * @retval None
  */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /*##-1- Configue LSE as RTC clock soucre ###################################*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState       = RCC_LSI_OFF;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    use_LSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv  = RTC_SYNCH_PREDIV_LSI;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSI;
  }

  else
  {
    /* We use LSE */
    use_LSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv  = RTC_SYNCH_PREDIV_LSE;
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
       - OutPutType     = Open Drain
   */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
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
  sdatestructure.Year    = 0x16;
  sdatestructure.Month   = RTC_MONTH_MARCH;
  sdatestructure.Date    = 0x21;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time */
  /* Set Time: 08:10:00 */
  stimestructure.Hours          = 0x08;
  stimestructure.Minutes        = 0x10;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Configures the current date
  * @param  y the year value to be set
  * @param  m the month value to be set
  * @param  d the day value to be set
  * @param  dw the day-week value to be set
  * @retval None
  */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year    = y;
  sdatestructure.Month   = m;
  sdatestructure.Date    = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Configures the current time
  * @param  hh the hour value to be set
  * @param  mm the minute value to be set
  * @param  ss the second value to be set
  * @retval None
  */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
    {
      magcal_request = 1;
    }
  }
}


/**
  * @brief  Period elapsed callback
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *              the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_FX)
  {
    sensor_read_request = 1;
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
