/**
  ******************************************************************************
  * @file    PMSM motor parameters.h
  * @author  STMCWB ver.4.2.0.15408
  * @version 4.1.0
  * @date    2016-23-03 17:15:22
  * @project STMF4_SINGLE.stmcx
  * @path    G:\software\Gimbal_2016_0323\rt-thread\bsp\stm32f40x
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *                 library v4.1.0
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

#define MOTOR_TYPE             PMSM

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          4 /* Number of motor pole pairs */
#define RS                     1.73 /* Stator resistance , ohm*/
#define LS                     0.000330 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the 
   PID for speed regulation (i.e. reference torque). 
   Transformation of real currents (A) into s16 format must be done accordingly with 
   formula:
   Phase current (s16 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         7685  
#define MOTOR_MAX_SPEED_RPM     2000 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  4.0 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG                -7685 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only 
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_AVAILABLE  FALSE
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the  
                                                 mechanical position of the sensors                    
                                                 withreference to an electrical cycle. 
                                                 It can be either DEGREES_120 or 
                                                 DEGREES_60 */
                                                                                   
#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees  
                                                 the electrical phase shift between 
                                                 the low to high transition of 
                                                 signal H1 and the maximum of 
                                                 the Bemf induced on phase A */ 
/*** Quadrature encoder ***/ 
#define ENCODER_AVAILABLE       TRUE
#define ENCODER_PPR             8192  /*!< Number of pulses per 
                                            revolution */

/*** Motor Profiler ***/
#define LDLQ_RATIO              1.0 /*!< Ld vs Lq ratio.*/
#define DC_CURRENT_RS_MEAS      0.50 /*!< Maxium level of DC current used
                                                      for RS measurement.*/

#endif /*__PMSM_MOTOR_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
