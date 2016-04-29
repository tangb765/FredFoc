/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PID_regulators.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of PI(D) related functions.
* 
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
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
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "app_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_int16_t PID_Regulator(rt_int16_t hReference, rt_int16_t hPresentFeedback, struct pid_struct_t *PID_Struct);

void PID_Torque_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Torque_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Torque_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor);

void PID_Flux_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Flux_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Flux_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor);

void PID_Speed_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Speed_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Speed_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor);

void PID_Position_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Position_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor);
void PID_Position_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor);
/* Exported variables ------------------------------------------------------- */

#endif 

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
