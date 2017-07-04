/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A2/LSM6DSL_MultiEvent/Src/main.c
 * @author  CL
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   Main program body
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

#include <string.h> /* strlen */
#include <stdio.h>  /* snprintf */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A2_Examples
 * @{
 */

/** @addtogroup MULTI_EVENT
 * @{
 */

extern int use_LSI;
int RTC_SYNCH_PREDIV;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint8_t mems_event_detected = 0;
static volatile uint8_t HW_event_enabled = 0;
static volatile uint8_t HW_event_enable_request  = 1;
static volatile uint8_t HW_event_disable_request = 0;
static char dataOut[MAX_BUF_SIZE];
static RTC_HandleTypeDef RtcHandle;
static uint16_t step_count = 0;

static void *LSM6DSL_X_0_handle = NULL;

/* Private function prototypes -----------------------------------------------*/

static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void RTC_Handler( void );
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void Send_Step_Count( void );
static void sendOrientation( void );

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to detect free fall, tap, double tap, tilt, wake up,
 *         6D Orientation and step events through the LSM6DSL sensor and send data from a Nucleo board using UART to a
 *         connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can simulate all the events and then view the data using an hyper terminal.
 *         - the user can push the button to enable/disable all hardware features.
 * @param  None
 * @retval Integer
 */
int main( void )
{

  ACCELERO_Event_Status_t status;

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED */
  BSP_LED_Init( LED2 );

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize UART */
  USARTConfig();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Initialize all sensors */
  initializeAllSensors();
  /* Enable all sensors */
  enableAllSensors();

  while (1)
  {
    if ( mems_event_detected != 0 )
    {
      mems_event_detected = 0;

      if ( BSP_ACCELERO_Get_Event_Status_Ext( LSM6DSL_X_0_handle, &status ) == COMPONENT_OK )
      {
        if ( status.StepStatus != 0 )
        {
          Send_Step_Count();
        }

        if ( status.FreeFallStatus != 0 )
        {
          snprintf( dataOut, MAX_BUF_SIZE, "Free fall detected!!!\r\n" );
          HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
        }

        if ( status.TapStatus != 0 )
        {
          snprintf( dataOut, MAX_BUF_SIZE, "Single Tap detected!!!\r\n" );
          HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
        }

        if ( status.DoubleTapStatus != 0 )
        {
          snprintf( dataOut, MAX_BUF_SIZE, "Double Tap detected!!!\r\n" );
          HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
        }

        if ( status.TiltStatus != 0 )
        {
          snprintf( dataOut, MAX_BUF_SIZE, "Tilt detected!!!\r\n" );
          HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
        }

        if ( status.D6DOrientationStatus != 0 )
        {
          sendOrientation();
        }

        if ( status.WakeUpStatus != 0 )
        {
          snprintf( dataOut, MAX_BUF_SIZE, "Wake Up detected!!!\r\n" );
          HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
        }
      }
    }

    if ( HW_event_enable_request != 0 )
    {
      if ( (BSP_ACCELERO_Enable_Pedometer_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_Tilt_Detection_Ext( LSM6DSL_X_0_handle, INT1_PIN ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_Free_Fall_Detection_Ext( LSM6DSL_X_0_handle, INT1_PIN ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_Single_Tap_Detection_Ext( LSM6DSL_X_0_handle, INT1_PIN ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_Double_Tap_Detection_Ext( LSM6DSL_X_0_handle, INT1_PIN ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_6D_Orientation_Ext( LSM6DSL_X_0_handle, INT1_PIN ) == COMPONENT_OK) && (BSP_ACCELERO_Enable_Wake_Up_Detection_Ext( LSM6DSL_X_0_handle, INT2_PIN ) == COMPONENT_OK) )
      {
        HW_event_enabled = 1;
        HW_event_enable_request = 0;
      }
    }

    if ( HW_event_disable_request != 0 )
    {
      if ( (BSP_ACCELERO_Disable_Pedometer_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_Tilt_Detection_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_Free_Fall_Detection_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_Single_Tap_Detection_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_Double_Tap_Detection_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_6D_Orientation_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) && (BSP_ACCELERO_Disable_Wake_Up_Detection_Ext( LSM6DSL_X_0_handle ) == COMPONENT_OK) )
      {
        HW_event_enabled = 0;
        HW_event_disable_request = 0;
      }
    }
  }
}

/**
 * @brief  Send actual 6D orientation to UART
 * @param  None
 * @retval None
 */
static void sendOrientation( void )
{

  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  uint8_t instance;

  BSP_ACCELERO_Get_Instance( LSM6DSL_X_0_handle, &instance );

  if ( BSP_ACCELERO_Get_6D_Orientation_XL_Ext( LSM6DSL_X_0_handle, &xl ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation XL axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_XH_Ext( LSM6DSL_X_0_handle, &xh ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation XH axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_YL_Ext( LSM6DSL_X_0_handle, &yl ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation YL axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_YH_Ext( LSM6DSL_X_0_handle, &yh ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation YH axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( LSM6DSL_X_0_handle, &zl ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation ZL axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( LSM6DSL_X_0_handle, &zh ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting 6D orientation ZH axis from LSM6DSL - accelerometer[%d].\r\n", instance );
    HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
    return;
  }

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |  *             | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |________________| \r\n" );
  }

  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |             *  | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |  *             | " \
             "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |                | " \
             "\r\n |             *  | " \
             "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  __*_____________  " \
             "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "\r\n  ________________  " \
             "\r\n |________________| " \
             "\r\n    *               \r\n" );
  }

  else
  {
    snprintf( dataOut, MAX_BUF_SIZE, "None of the 6D orientation axes is set in LSM6DSL - accelerometer[%d].\r\n", instance );
  }

  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  Handles the time+date getting/sending
 * @param  None
 * @retval None
 */
static void RTC_Handler( void )
{

  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime( &RtcHandle, &stimestructure, FORMAT_BIN );
  HAL_RTC_GetDate( &RtcHandle, &sdatestructureget, FORMAT_BIN );
  subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) / ( RTC_SYNCH_PREDIV + 1 )) & \
            0xff );

  /* First send the extra line separately to clean the UART line (better results). */
  snprintf( dataOut, MAX_BUF_SIZE, "\r\n" );
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );

  snprintf( dataOut, MAX_BUF_SIZE, "Time stamp: %02d:%02d:%02d.%02d\r\n", stimestructure.Hours, stimestructure.Minutes, \
           stimestructure.Seconds, subSec );
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{

  if(BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle ) == COMPONENT_ERROR)
  {
    /* LSM6DSL not detected, switch on LED2 and go to infinity loop */
    BSP_LED_On( LED2 );
    while (1)
    {}
  }
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
}



/**
 * @brief  Send time stamp and step count to UART
 * @param  None
 * @retval None
 */
static void Send_Step_Count( void )
{
  uint8_t instance;

  RTC_Handler();

  BSP_ACCELERO_Get_Instance( LSM6DSL_X_0_handle, &instance );

  if ( BSP_ACCELERO_Get_Step_Count_Ext( LSM6DSL_X_0_handle, &step_count ) == COMPONENT_ERROR )
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Error getting step count from LSM6DSL - accelerometer[%d].\r\n", instance );
  }
  else
  {
    snprintf( dataOut, MAX_BUF_SIZE, "Step count: %d\r\n", step_count );
  }

  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config( void )
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
static void RTC_TimeStampConfig( void )
{

  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-3- Configure the Date using BCD format ################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure the Time using BCD format#################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
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
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{

  /* User button. */
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_RESET )
    {
      // Toggle enable/disable HW events (available only for LSM6DSL sensor).
      if ( HW_event_enabled != 0 )
      {
        HW_event_disable_request = 1;
      }
      else
      {
        HW_event_enable_request = 1;
      }
    }
  }

  /* Hardware event (available only for LSM6DSL sensor). */
  else if ( GPIO_Pin == LSM6DSL_INT1_O_PIN || GPIO_Pin == LSM6DSL_INT2_O_PIN )
  {
    mems_event_detected = 1;
  }
}



/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler( void )
{

  while (1)
  {}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.0
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
