/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_pwm_3shunt_prm.h
* Author             : FOCSDK development tool ver. 1.0.0 by IMS Systems Lab
* Creation date      : Wed Sep 23 16:41:26 2015
* Description        : Contains the list of project specific parameters related
*                      to the three-shunt current reading.
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
#ifndef __MC_PWM_3SHUNT_PRM_H
#define __MC_PWM_3SHUNT_PRM_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((rt_uint32_t)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

	#define PWM_PRSC ((rt_uint8_t)0)

        /* Resolution: 1Hz */                            
	#define PWM_PERIOD ((rt_uint16_t) (CKTIM / (rt_uint32_t)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
	#define DEADTIME  (rt_uint16_t)((unsigned long long)CKTIM/2 \
          *(unsigned long long)DEADTIME_NS/1000000000uL) 

///////////////////////////// Current reading parameters //////////////////////

//#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
//#define PHASE_A_GPIO_PORT       GPIOC
//#define PHASE_A_GPIO_PIN        GPIO_Pin_1

//#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
//#define PHASE_B_GPIO_PORT       GPIOC
//#define PHASE_B_GPIO_PIN        GPIO_Pin_2

//#define PHASE_C_ADC_CHANNEL     ADC_Channel_13
//#define PHASE_C_GPIO_PORT       GPIOC
//#define PHASE_C_GPIO_PIN        GPIO_Pin_3

//#define SAMPLING_TIME_NS   200  //200ns
#define SAMPLING_TIME_NS   700  //700ns
//#define SAMPLING_TIME_NS  1200  //1.2us
//#define SAMPLING_TIME_NS  2450  //2.45us

#if (SAMPLING_TIME_NS == 200)
#define SAMPLING_TIME_CK  ADC_SampleTime_1Cycles5
#elif (SAMPLING_TIME_NS == 700)
#define SAMPLING_TIME_CK  ADC_SampleTime_7Cycles5
#elif (SAMPLING_TIME_NS == 1200)
#define SAMPLING_TIME_CK  ADC_SampleTime_13Cycles5
#elif (SAMPLING_TIME_NS == 2450)
#define SAMPLING_TIME_CK  ADC_SampleTime_28Cycles5
#else
#warning "Sampling time is not a possible value"
#endif

#define TNOISE_NS 2550
#define TRISE_NS 2550

#define SAMPLING_TIME (rt_uint16_t)(((rt_uint16_t)(SAMPLING_TIME_NS) * 72uL)/1000uL) 
#define TNOISE (rt_uint16_t)((((rt_uint16_t)(TNOISE_NS)) * 72uL)/1000uL)
#define TRISE (rt_uint16_t)((((rt_uint16_t)(TRISE_NS)) * 72uL)/1000uL)
#define TDEAD (rt_uint16_t)((DEADTIME_NS * 72uL)/1000uL)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define TW_AFTER ((rt_uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*72ul)/1000ul))
#define TW_BEFORE (((rt_uint16_t)(((((rt_uint16_t)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)

/////////////////  Power Stage management Conversions setting ////////////////////////

#define TEMP_FDBK_CHANNEL                 ADC_Channel_10
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOC
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_0

#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

#endif  /*__MC_PWM_3SHUNT_PRM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
