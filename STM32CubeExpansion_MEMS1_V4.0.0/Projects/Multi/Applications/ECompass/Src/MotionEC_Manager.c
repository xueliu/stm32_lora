/**
  ******************************************************************************
  * @file    MotionEC_Manager.c
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-May-2017
  * @brief   E-Compass management
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
#include <math.h>
#include "MotionEC_Manager.h"

/** @addtogroup MOTION_EC_Applications
  * @{
  */

/** @addtogroup E_COMPASS
  * @{
  */

/** @addtogroup MotionEC_Manager MotionEC_Manager
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI  (3.141593f)
#endif

/* Public variables ----------------------------------------------------------*/
extern SensorAxes_t ACC_Value;      /* Raw accelerometer data [mg] */
extern SensorAxes_t MAG_ValueComp;  /* Compensated magnetometer data [mGauss] */

/* Private variables ---------------------------------------------------------*/

/* Transformation matrices: XYZ frame -> ENU frame
 * Note: These matrices describe how to transform on-board orientation of each
 * individual sensor component to common East-North-Up (ENU) orientation of
 * whole device.
 */
#if (defined (USE_IKS01A1))
static float LSM6DS0_0_Matrix[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // ACC, GYRO
static float LIS3MDL_0_Matrix[3][3] = {{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}; // MAG

#elif (defined (USE_IKS01A2))
// static float LSM6DSL_0_Matrix[3][3]   = {{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}; // ACC, GYRO
static float LSM303AGR_0_Matrix[3][3] = {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}; // ACC, MAG

#else
#error "ERROR: Unknown MEMS shield!"
#endif

/* Function prototypes -------------------------------------------------------*/
static int calc_heading(float *heading, float v_head[]);
static void q_conjug(float q_conj[], float q_src[]);
static void q_multiply(float q_res[], float q_a[], float q_b[]);
static void transform_xyz_to_enu(SensorAxes_t *input, float output[], float matrix[][3]);
static void v_rotate(float v_new[], float q_rot[], float v_old[]);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize and reset the MotionEC engine
  * @param  freq  Sensors sampling frequency [Hz]
  * @retval none
  */
void MotionEC_manager_init(float freq)
{
  MotionEC_Initialize(freq);
  MotionEC_SetOrientationEnable(MEC_ENABLE);
  MotionEC_SetVirtualGyroEnable(MEC_ENABLE);
  MotionEC_SetGravityEnable(MEC_ENABLE);
  MotionEC_SetLinearAccEnable(MEC_ENABLE);
}

/**
  * @brief  Run E-Compass algorithm (accelerometer and magnetometer data fusion)
  * @param  Msg  Data stream
  * @retval none
  */
void MotionEC_manager_run(TMsg *Msg)
{
  MEC_input_t  data_in;
  MEC_output_t data_out;

  /* Do sensor orientation transformation */
#if (defined (USE_IKS01A1))
  transform_xyz_to_enu(&ACC_Value, data_in.Acc, LSM6DS0_0_Matrix);
  transform_xyz_to_enu(&MAG_ValueComp, data_in.Mag, LIS3MDL_0_Matrix);

#elif (defined (USE_IKS01A2))
  transform_xyz_to_enu(&ACC_Value, data_in.Acc, LSM303AGR_0_Matrix);
  transform_xyz_to_enu(&MAG_ValueComp, data_in.Mag, LSM303AGR_0_Matrix);

#else
#error "ERROR: Unknown MEMS shield!"
#endif

  /* Raw accelerometer data [g] */
  data_in.Acc[0] = data_in.Acc[0] / 1000.0f; /* East */
  data_in.Acc[1] = data_in.Acc[1] / 1000.0f; /* North */
  data_in.Acc[2] = data_in.Acc[2] / 1000.0f; /* Up */

  /* Compensated magnetometer data [uT / 50], [mGauss / 5] */
  data_in.Mag[0] = data_in.Mag[0] / 5.0f; /* East */
  data_in.Mag[1] = data_in.Mag[1] / 5.0f; /* North */
  data_in.Mag[2] = data_in.Mag[2] / 5.0f; /* Up */

  /* Delta time [s] */
  data_in.DTime = REPORT_INTERVAL / 1000.0f;

  /* Do calculation */
  MotionEC_Run(&data_in, &data_out);

  /* Write data to output stream */
  FloatToArray(&Msg->Data[55], data_out.Quaternion[0]);
  FloatToArray(&Msg->Data[59], data_out.Quaternion[1]);
  FloatToArray(&Msg->Data[63], data_out.Quaternion[2]);
  FloatToArray(&Msg->Data[67], data_out.Quaternion[3]);

  FloatToArray(&Msg->Data[71], data_out.Euler[0]);
  FloatToArray(&Msg->Data[75], data_out.Euler[1]);
  FloatToArray(&Msg->Data[79], data_out.Euler[2]);

  FloatToArray(&Msg->Data[83], data_out.IGyro[0]);
  FloatToArray(&Msg->Data[87], data_out.IGyro[1]);
  FloatToArray(&Msg->Data[91], data_out.IGyro[2]);

  FloatToArray(&Msg->Data[95], data_out.Gravity[0]);
  FloatToArray(&Msg->Data[99], data_out.Gravity[1]);
  FloatToArray(&Msg->Data[103], data_out.Gravity[2]);

  FloatToArray(&Msg->Data[107], data_out.Linear[0]);
  FloatToArray(&Msg->Data[111], data_out.Linear[1]);
  FloatToArray(&Msg->Data[115], data_out.Linear[2]);

  float v_base[3] = {0.0, 1.0, 0.0};
  float v_head[3];
  float heading;
  int heading_valid = 0;

  v_rotate(v_head, data_out.Quaternion, v_base);
  heading_valid = calc_heading(&heading, v_head);

  FloatToArray(&Msg->Data[119], heading);
  Msg->Data[123] = (uint8_t)heading_valid;
}

/**
  * @brief  Get the library version
  * @param  version  Library version string (must be array of 35 char)
  * @param  length  Library version string length
  * @retval none
  */
void MotionEC_manager_get_version(char *version, int *length)
{
  *length = MotionEC_GetLibVersion(version);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate heading.
  * @param  heading Device heading in range <0, 360) degrees
  * @param  v_head  Device orientation vector for heading
  * @retval 1 in case of success, 0 otherwise
  */
static int calc_heading(float *heading, float v_head[])
{
  const float tol_deg = 5.0; /* Tolerance [deg] */
  float tolerance = sin(tol_deg * M_PI / 180.0f);

  if ( v_head[0] > (-1) * tolerance && v_head[0] < tolerance
    && v_head[1] > (-1) * tolerance && v_head[1] < tolerance)
  {
    *heading = 0.0;
    return 0; /* Device is pointing up or down - it is impossible to evaluate heading */
  }

  else
  {
    *heading = atan2(v_head[0], v_head[1]) * 180.0f / M_PI;
    *heading = floor(*heading * 100.0f  +  0.5f) / 100.0f;         /* Rounds number to two decimal digits */
    *heading = (*heading < 0.0f) ? (*heading + 360.0f) : *heading; /* Change negative value to be in range <0,360) */
    return 1;
  }
}

/**
  * @brief  Create conjugated quaternion.
  * @param  q_conj  Conjugated quaternion
  * @param  q_src   Source quaternion
  * @retval None
  */
static void q_conjug(float q_conj[], float q_src[])
{
  q_conj[0] = (-1) * q_src[0];
  q_conj[1] = (-1) * q_src[1];
  q_conj[2] = (-1) * q_src[2];
  q_conj[3] =        q_src[3];

  return;
}

/**
  * @brief  Do quaternion multiplication.
  * @param  q_res  Quaternion multiplication result: q_res = q_a q_b
  * @param  q_a    Quaternion A
  * @param  q_b    Quaternion B
  * @retval None
  */
static void q_multiply(float q_res[], float q_a[], float q_b[])
{
  q_res[0] =
      q_a[3] * q_b[0]
    + q_a[0] * q_b[3]
    + q_a[1] * q_b[2]
    - q_a[2] * q_b[1]
  ;

  q_res[1] =
      q_a[3] * q_b[1]
    + q_a[1] * q_b[3]
    + q_a[2] * q_b[0]
    - q_a[0] * q_b[2]
  ;

  q_res[2] =
      q_a[3] * q_b[2]
    + q_a[2] * q_b[3]
    + q_a[0] * q_b[1]
    - q_a[1] * q_b[0]
  ;

  q_res[3] =
      q_a[3] * q_b[3]
    - q_a[0] * q_b[0]
    - q_a[1] * q_b[1]
    - q_a[2] * q_b[2]
  ;

  return;
}

/**
  * @brief  Transform from X, Y, Z (XYZ) frame to East, North, Up (ENU) frame.
  * @param  input
  * @retval None
  */
static void transform_xyz_to_enu(SensorAxes_t *input, float output[], float matrix[][3])
{
  output[0] = matrix[0][0] * input->AXIS_X  +  matrix[0][1] * input->AXIS_Y  +  matrix[0][2] * input->AXIS_Z;
  output[1] = matrix[1][0] * input->AXIS_X  +  matrix[1][1] * input->AXIS_Y  +  matrix[1][2] * input->AXIS_Z;
  output[2] = matrix[2][0] * input->AXIS_X  +  matrix[2][1] * input->AXIS_Y  +  matrix[2][2] * input->AXIS_Z;

  return;
}

/**
  * @brief  Rotate vector using quaternion. Uses following equation:
  *           v_new = q_rot v_old q_rot_inv
  *
  * @param  v_new  Vector after rotation
  * @param  q_rot  Rotation quaternion
  * @param  v_old  Vector before rotation
  * @retval None
  */
static void v_rotate(float v_new[], float q_rot[], float v_old[])
{
  float q_old[4];
  float q_new[4];
  float q_rot_inv[4];
  float q_temp[4];

  /* Create quaternion from old position vector */
  q_old[0] = v_old[0];
  q_old[1] = v_old[1];
  q_old[2] = v_old[2];
  q_old[3] = 0;

  q_conjug(q_rot_inv, q_rot);
  q_multiply(q_temp, q_old, q_rot_inv);
  q_multiply(q_new, q_rot, q_temp);

  v_new[0] = q_new[0];
  v_new[1] = q_new[1];
  v_new[2] = q_new[2];

  return;
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
