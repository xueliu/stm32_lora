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
  * @mainpage Documentation for MotionFA package of X-CUBE-MEMS1 Software for
  * X-NUCLEO-IKS01A1 and X-NUCLEO-IKS01A2 expansion board
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * MotionFA software is an add-on for the X-CUBE-MEMS1 software and provides
  * real-time fitness activities recognition data.
  * The expansion is built on top of STM32Cube software technology that eases
  * portability across different STM32 microcontrollers.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "com.h"
#include "DemoDatalog.h"
#include "DemoSerial.h"
#include "MotionFA_Manager.h"

/** @addtogroup MOTION_FA_Applications
  * @{
  */

/** @addtogroup FITNESS_ACTIVITIES
  * @{
  */

/** @addtogroup Main Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SAMPLE_FREQ          ((uint8_t)50)                       /* [Hz] */
#define SESSION_COUNTER_MAX  ((uint32_t)(5 * 60 * SAMPLE_FREQ))  /* equals to 5 min */

/* Public variables ----------------------------------------------------------*/
volatile uint8_t flash_erase_request = 0;

program_state_t ProgramState = GUI_MODE;
flash_state_t   FlashState   = FLASH_READY;

uint8_t DataLoggerActive = 0;
uint32_t Sensors_Enabled = 0;

int use_LSI = 0;

SensorAxes_t ACC_Value;
float PRESSURE_Value;

void *ACCELERO_handle    = NULL;
void *GYRO_handle        = NULL;
void *MAGNETO_handle     = NULL;
void *HUMIDITY_handle    = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle    = NULL;

TMsg Msg;
TIM_HandleTypeDef FA_TimHandle;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile static uint8_t sensor_read_request     = 0;
volatile static uint8_t gui_mode_request        = 0;
volatile static uint8_t standalone_mode_request = 0;

static uint8_t SessionNew    = 1;
static uint8_t FA_Data_index = 0;

static RTC_HandleTypeDef RtcHandle;

static int RTC_SYNCH_PREDIV;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_FA_Init(void);
static void RTC_Handler(TMsg *Msg);
static void RTC_Handler_FAStamp(DataTime_t *stamp);
static void FA_Data_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);

/* Public functions ----------------------------------------------------------*/
/**
 * @brief   Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_OLA01A2
 *          expansion board to count number of bices curls or squats and send it from a
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

  /* Fitness Activities API initialization function */
  MotionFA_manager_init(ACCELERO_handle);

  /* OPTIONAL */
  /* Get library version */
  MotionFA_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log */
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Timer for FA algorithm synchronization Initialization */
  MX_TIM_FA_Init();

  /* Set current data storage address to the first free address in flash memory */
  FlashState = Datalog_SetAddress();
  if (FlashState == FLASH_FULL)
  {
    BSP_LED_On(LED2);
  }

  else {
    BSP_LED_On(LED2);
    HAL_Delay(500);
    BSP_LED_Off(LED2);
  }

  while(1)
  {
    if ((UART_ReceivedMSG((TMsg*)&Msg)) && (Msg.Data[0] == DEV_ADDR))
      if (ProgramState == GUI_MODE)
        HandleMSG((TMsg*)&Msg);

    if (flash_erase_request)
    {
      flash_erase_request = 0;

      BSP_LED_Off(LED2);

      Datalog_FlashErase();

      for (int i = 0; i < 3; i++)
      {
        BSP_LED_Toggle(LED2);
        HAL_Delay(500);
        BSP_LED_Toggle(LED2);
        HAL_Delay(1000);
      }

      FlashState = FLASH_READY;
    }

    if (gui_mode_request)
    {
      gui_mode_request = 0;

      HAL_TIM_Base_Stop_IT(&FA_TimHandle);
      BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
      Sensors_Enabled &= ~ACCELEROMETER_SENSOR;
      BSP_PRESSURE_Sensor_Disable(PRESSURE_handle);
      Sensors_Enabled &= ~PRESSURE_SENSOR;

      /* Store in FLASH any Fitness Activities data collected during STANDALONE mode
         remaining in buffer */
      Datalog_SaveFA2Mem(FA_Data_index);

      ProgramState = GUI_MODE;
    }

    if (standalone_mode_request)
    {
      standalone_mode_request = 0;

      SessionNew = 1;

      BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
      Sensors_Enabled |= ACCELEROMETER_SENSOR;
      BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
      Sensors_Enabled |= PRESSURE_SENSOR;
      HAL_TIM_Base_Start_IT(&FA_TimHandle);

      ProgramState = STANDALONE_MODE;
    }

    if (sensor_read_request)
    {
      sensor_read_request = 0;

      switch (ProgramState)
      {
      case GUI_MODE:

        /* Acquire data from enabled sensors and fill Msg stream */
        RTC_Handler(&Msg);
        Accelero_Sensor_Handler(&Msg);
        Gyro_Sensor_Handler(&Msg);
        Magneto_Sensor_Handler(&Msg);
        Humidity_Sensor_Handler(&Msg);
        Temperature_Sensor_Handler(&Msg);
        Pressure_Sensor_Handler(&Msg);

        /* Fitness Activities specific part */
        FA_Data_Handler(&Msg);

        /* Send data stream */
        INIT_STREAMING_HEADER(&Msg);
        Msg.Len = STREAMING_MSG_LENGTH;
        UART_SendMsg(&Msg);
        break;

      case STANDALONE_MODE:
        Accelero_Sensor_Handler(&Msg);
        Pressure_Sensor_Handler(&Msg);
        FA_Data_Handler(&Msg);
        break;

      default:
        Error_Handler();
      }
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
  /* Initialize and Configure Accelerometer and Pressure Sensor: ODR >= 25 Hz */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, 25.0f);

  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
  BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle, 25.0f);

  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  /* Configure ODR due link between accelerometer and gyroscope in to LSM6DS0 */
  BSP_GYRO_Set_ODR_Value(GYRO_handle, 25.0f);

  /* Initialize other sensors (not used in STANDALONE mode) */
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);
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
  * @brief  TIM_FA init function.
  * @param  None
  * @retval None
  * @details This function intialize the Timer used to syncronize the FA algorithm
  */
static void MX_TIM_FA_Init(void)
{
#define PERIOD_25HZ  ((uint8_t)78)

#if (defined (USE_STM32F4XX_NUCLEO))    /* 84 MHZ CPU clock */
#define PRESCALER_25HZ  ((uint16_t)41999)

#elif (defined (USE_STM32L4XX_NUCLEO))  /* 80 MHZ CPU clock */
#define PRESCALER_25HZ  ((uint16_t)39999)

#else
#error Not supported platform
#endif

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  FA_TimHandle.Instance = TIM_FA;
  FA_TimHandle.Init.Prescaler = PRESCALER_25HZ;
  FA_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  FA_TimHandle.Init.Period = PERIOD_25HZ;
  FA_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&FA_TimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&FA_TimHandle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&FA_TimHandle, &sMasterConfig);
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
  * @brief  Handles the Fitness Activities data timestamp
  * @param  Stamp - Stamp (time + date)
  * @retval None
  */
static void RTC_Handler_FAStamp(DataTime_t *stamp)
{
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;

  HAL_RTC_GetTime(&RtcHandle, &time, FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &date, FORMAT_BIN);

  stamp->Date[0] = (uint8_t)date.Month;
  stamp->Date[1] = (uint8_t)date.Date;
  stamp->Date[2] = (uint8_t)date.Year;
  stamp->Time[0] = (uint8_t)time.Hours;
  stamp->Time[1] = (uint8_t)time.Minutes;
  stamp->Time[2] = (uint8_t)time.Seconds;
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
  SensorAxes_t GYR_Value;

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
  SensorAxes_t MAG_Value;

  if (Sensors_Enabled & MAGNETIC_SENSOR)
  {
    if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
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
  * @brief  Fitness Activities data handler
  * @param  Msg - Pose Estimation data part of the stream
  * @retval None
  */
static void FA_Data_Handler(TMsg *Msg)
{
  MFA_input_t data_in;
  static MFA_output_t activity_prev = {0};
  static MFA_output_t activity;
  MFA_activity_t  activity_type;

  static uint32_t SessionCounter = 0;

  if (Sensors_Enabled & ACCELEROMETER_SENSOR)
  {
    /* Convert acceleration from [mg] to [g] */
    data_in.AccX = (float)ACC_Value.AXIS_X / 1000.0f;
    data_in.AccY = (float)ACC_Value.AXIS_Y / 1000.0f;
    data_in.AccZ = (float)ACC_Value.AXIS_Z / 1000.0f;

    data_in.Press = PRESSURE_Value;

    /* Run Pose Estimation algorithm */
    MotionFA_manager_run(&data_in, &activity);
    MotionFA_manager_get_activity(&activity_type);

    if (ProgramState == STANDALONE_MODE)
    {
      BSP_LED_On(LED2);
      HAL_Delay(1);
      BSP_LED_Off(LED2);
    }

    if (ProgramState == GUI_MODE)
    {
      Serialize_s32(&Msg->Data[55], (int32_t)(activity.Counter), 4);
      Serialize_s32(&Msg->Data[59], (int32_t)(activity_type + 1), 4);
    }

    else if (ProgramState == STANDALONE_MODE)
    {
      /* New session */
      if (SessionNew == 1)
      {
        SessionNew     = 0;
        SessionCounter = 0;
        FA_Data_index  = 0;

        /* Create new session record with only date and time */
        RTC_Handler_FAStamp(&DataByte_FA[FA_Data_index].DateTime);
        DataByte_FA[FA_Data_index].ActivityType = 0;
        DataByte_FA[FA_Data_index].ActivityCounter  = 0;
        FA_Data_index++;

	activity_prev.Counter = 0;
        return;
      }

      /* Buffer full */
      if (FA_Data_index >= DATABYTE_FA_LEN)
      {
        /* Pose estimation data are stored in FLASH */
        if (Datalog_SaveFA2Mem(FA_Data_index) == 0)
          gui_mode_request = 1;

        FA_Data_index = 0;
      }

      /* Position changed */
      if (activity.Counter != activity_prev.Counter)
      {
        /* Store new pose in buffer */
        RTC_Handler_FAStamp(&DataByte_FA[FA_Data_index].DateTime);
        DataByte_FA[FA_Data_index].ActivityType = activity_type + 1;
        DataByte_FA[FA_Data_index].ActivityCounter  = activity.Counter;
        FA_Data_index++;

        activity_prev.Counter = activity.Counter;
      }

      /* Periodic Carry Position data save */
      if (SessionCounter >= SESSION_COUNTER_MAX)
      {
        /* Pose estimation data are stored in FLASH */
        Datalog_SaveFA2Mem(FA_Data_index);
        SessionCounter = 0;
        FA_Data_index  = 0;
      }

      else
      {
        SessionCounter++;
      }
    }

    else
    {
      Error_Handler();
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
      if (ProgramState == GUI_MODE)
        standalone_mode_request = 1;

      else if (ProgramState == STANDALONE_MODE)
        gui_mode_request = 1;

      else
        Error_Handler();
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
  if (htim->Instance == TIM_FA)
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
