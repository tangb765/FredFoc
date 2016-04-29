/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_MClib.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file gathers the motor control header files which 
*                      are needed depending on configuration.
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
#ifndef __STM32F10xMCLIB_H
#define __STM32F10xMCLIB_H

/* Includes ------------------------------------------------------------------*/

#include "MC_type.h"

#include "stm32f10x_encoder.h"
#define GET_ELECTRICAL_ANGLE    ENC_Get_Electrical_Angle()
#define GET_MECHANICAL_ANGLE		ENC_Get_Mechanical_Angle()

//#define GET_SPEED_0_1HZ         ENC_Get_Mechanical_Speed()	
#define GET_SPEED_DPS					ENC_Calc_Average_Speed()
//#define GET_SPEED_DPP    (rt_int16_t)((ENC_Get_Mechanical_Speed()*_0_1HZ_2_128DPP)/128)

#include "stm32f10x_svpwm_3shunt.h"
#define GET_PHASE_CURRENTS SVPWM_3ShuntGetPhaseCurrentValues
#define CALC_SVPWM SVPWM_3ShuntCalcDutyCycles

#include "MC_Clarke_Park.h"
#include "MC_FOC_Drive.h"
//#include "MC_PID_regulators.h"
#include "stm32f10x_Timebase.h"
#include "MC_PMSM_motor_param.h"
#include "MC_MotorControl_Layer.h"

int MClib_application_init(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10xMCLIB_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
