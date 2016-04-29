/**
  ******************************************************************************
  * @file    PMSM motor 2 parameters.h
  * @author  STMCWB ver.4.0.0.14274
  * @version 4.0.0
  * @date    2015-11-09 00:01:56
  * @project SDK40x-STEVAL-IHM042V1-Shinano-DUAL-DRIVE.stmc
  * @path    E:\20150404\STM32 PMSM FOC LIBv4.0\Web\STM32 PMSM FOC LIBv4.0 gui
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *                 library v4.0.0
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#ifndef __PMSM_MOTOR2_PARAMETERS_H
#define __PMSM_MOTOR2_PARAMETERS_H

#define MOTOR_TYPE2             PMSM

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM2            7  /* Number of motor pole pairs */
#define RS2                       2.34  /* Stator resistance , ohm */
#define LS2                       0.00076  /* Stator inductance , H */
													/* For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the 
   PID for speed regulation (i.e. reference torque). 
   Transformation of real currents (A) into s16 format must be done accordingly with 
   formula:
   Phase current (s16 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT2         13440  
#define MOTOR_MAX_SPEED_RPM2     4000   /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT2  2.9   /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG2                -13440   /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only 
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_AVAILABLE2  FALSE
#define HALL_SENSORS_PLACEMENT2  DEGREES_120 /*!<Define here the  
                                                 mechanical position of the sensors                    
                                                 withreference to an electrical cycle. 
                                                 It can be either DEGREES_120 or 
                                                 DEGREES_60 */
                                                                                   
#define HALL_PHASE_SHIFT2        300 /*!< Define here in degrees  
                                                 the electrical phase shift between 
                                                 the low to high transition of 
                                                 signal H1 and the maximum of 
                                                 the Bemf induced on phase A */ 
/*** Quadrature encoder ***/ 
#define ENCODER_AVAILABLE2      TRUE 
#define ENCODER_PPR2            4096  /*!< Number of pulses per 
                                            revolution */
#endif /*__PMSM_MOTOR2_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
