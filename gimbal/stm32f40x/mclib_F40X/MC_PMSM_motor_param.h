/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PMSM_motor_param.h
* Author             : FOCSDK development tool ver. 1.0.0 by IMS Systems Lab
* Creation date      : Wed Sep 23 16:41:26 2015
* Description        : This file contains the PM motor parameters.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
* 28/08/08 v2.0.2
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAM_H
#define __MOTOR_PARAM_H

// Number of motor pair of poles
#define	POLE_PAIR_NUM 	(rt_uint8_t) 7        /* Number of motor pole pairs */
#define RS               1.73            /* Stator resistance , ohm*/
#define LS               0.00033        /* Stator inductance , H */

// When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the 
// PID for speed regulation (i.e. reference torque). 
// Whit MB459 board, the value must be calculated accordingly with formula:
// NOMINAL_CURRENT = (Nominal phase current (A, 0-to-peak)*32767* Rshunt) /0.64

//#define NOMINAL_CURRENT             (rt_int16_t)19858    //motor nominal current (0-pk)
#define NOMINAL_CURRENT             (rt_int16_t)12000//7943    //motor nominal current (0-pk)
#define MOTOR_MAX_SPEED_RPM         (rt_int16_t)2000   //maximum speed required
#define MOTOR_MIN_SPEED_RPM						(rt_int16_t)( -2000 )
//#define MOTOR_VOLTAGE_CONSTANT      4.0   //Volts RMS ph-ph /kRPM
//Demagnetization current
#define ID_DEMAG    -NOMINAL_CURRENT

#define IQMAX NOMINAL_CURRENT

///*not to be modified*/
//#define MAX_BEMF_VOLTAGE  (rt_uint16_t)((MOTOR_MAX_SPEED_RPM*\
//                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000*SQRT_3))

//#define MOTOR_MAX_SPEED_HZ (rt_int16_t)((MOTOR_MAX_SPEED_RPM*10)/60)
#define MOTOR_MAX_SPEED_HZ (rt_int16_t)((MOTOR_MAX_SPEED_RPM*6))

#define _0_1HZ_2_128DPP (rt_uint16_t)((POLE_PAIR_NUM*65536*128)/(SAMPLING_FREQ*10))

#endif /*__MC_PMSM_MOTOR_PARAM_H*/
/**************** (c) 2007  STMicroelectronics ********************************/
