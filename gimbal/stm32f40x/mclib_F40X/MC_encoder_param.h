/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_encoder_param.h
* Author             : FOCSDK development tool ver. 1.0.0 by IMS Systems Lab
* Creation date      : Wed Sep 23 16:41:26 2015
* Description        : Contains the list of project specific parameters related
*                      to the encoder speed and position feedback.
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
#ifndef __MC_ENCODER_PARAM_H
#define __MC_ENCODER_PARAM_H


/*****************************  Encoder settings ******************************/
#define ENCODER_PPR           (rt_uint16_t)(8192)   // number of pulses per revolution

/* Define here the absolute value of the application minimum and maximum speed 
                                                                   in rpm unit*/
#define MINIMUM_MECHANICAL_SPEED_RPM  (rt_int16_t)-2000   //rpm
#define MAXIMUM_MECHANICAL_SPEED_RPM  (rt_int16_t)2000 //rpm

/* Define here the number of consecutive error measurement to be detected 
   before going into FAULT state */
#define MAXIMUM_ERROR_NUMBER (rt_uint8_t)3
/* Computation Parameter*/
//Number of averaged speed measurement
#define SPEED_BUFFER_SIZE   16   // power of 2 required to ease computations

/*************************** Alignment settings *******************************/
//Alignemnt duration
#define T_ALIGNMENT           (rt_uint16_t) 4000    // Alignment time in ms

#define ALIGNMENT_ANGLE       (rt_uint16_t) 0 //Degrees [0..359] 
//  90° <-> Ia = I_ALIGNMENT, Ib = Ic =-I_ALIGNMENT/2) 

// With MB459 and ALIGNMENT_ANGLE equal to 90° 
//final alignment phase current = (I_ALIGNMENT * 0.64)/(32767 * Rshunt) 
//#define I_ALIGNMENT           (rt_uint16_t) 19858
#define I_ALIGNMENT           (rt_uint16_t) 6000 //0.25A

//Do not be modified
#define T_ALIGNMENT_PWM_STEPS     (rt_uint32_t) ((T_ALIGNMENT*SAMPLING_FREQ)/1000) 
#define ALIGNMENT_ANGLE_S16       (rt_int16_t)((rt_int32_t)(ALIGNMENT_ANGLE) * 65536/360)

#endif  /*__MC_ENCODER_PARAM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
