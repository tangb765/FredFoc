/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_type.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This header file provides structure type definitions that 
*                      are used throughout this motor control library.
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
#ifndef __MC_TYPE_H
#define __MC_TYPE_H
#include <rtthread.h>
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct 
{
  rt_int16_t qI_Component1;
  rt_int16_t qI_Component2;
} Curr_Components;

typedef struct 
{
  rt_int16_t qV_Component1;
  rt_int16_t qV_Component2;
} Volt_Components;

typedef struct
{
  rt_int16_t hCos;
  rt_int16_t hSin;
} Trig_Components;

typedef struct
{
 rt_int16_t hC1;
 rt_int16_t hC2;
 rt_int16_t hC3;
 rt_int16_t hC4;
 rt_int16_t hC5;
 rt_int16_t hC6;
 rt_int16_t hF1;
 rt_int16_t hF2;
 rt_int16_t hF3;
 rt_int16_t PLL_P;
 rt_int16_t PLL_I;
 rt_int32_t wMotorMaxSpeed_dpp;
 rt_uint16_t hPercentageFactor;
} StateObserver_Const; 

typedef struct
{
  rt_int16_t PLL_P;
  rt_int16_t PLL_I;
  rt_int16_t hC2;
  rt_int16_t hC4;
} StateObserver_GainsUpdate;

typedef struct
{
 rt_int16_t hsegdiv;
 rt_int32_t wangc[8];
 rt_int32_t wofst[8];
} MTPA_Const;

typedef enum 
{
IDLE, INIT, START, RUN, STOP, BRAKE, WAIT, FAULT
} SystStatus_t;

typedef enum 
{
NO_FAULT, OVER_VOLT, UNDER_VOLT
} BusV_t;


typedef struct 
{  
	rt_int16_t Mec_Angle;
	rt_int16_t Mec_Speed;
	rt_int16_t TLE5012_Mec_Speed;
	rt_int16_t Id_Torque;
	rt_int16_t Iq_Flux;
} MCLIB_Measure_variale_Struct_t;



#define U8_MAX     ((rt_uint8_t)255)
#define S8_MAX     ((rt_int8_t)127)
#define S8_MIN     ((rt_int8_t)-128)
#define U16_MAX    ((rt_uint16_t)65535u)
#define S16_MAX    ((rt_int16_t)32767)
#define S16_MIN    ((rt_int16_t)-32768)
#define U32_MAX    ((rt_uint32_t)4294967295uL)
#define S32_MAX    ((rt_int32_t)2147483647)
#define S32_MIN    ((rt_int32_t)-2147483648)

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
