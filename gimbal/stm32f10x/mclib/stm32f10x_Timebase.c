/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This module handles time base. It used in display and 
*                      fault management, speed regulation, motor ramp-up  
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
/* Includes ------------------------------------------------------------------*/


/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stdlib.h"
#include "board.h"
#include "app_protocol.h"
#include "app_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile rt_uint8_t bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
static rt_uint16_t bPID_Position_Sampling_Time_500us = PID_POSITION_SAMPLING_TIME;

/*******************************************************************************
* Function Name  : TB_Init
* Description    : TimeBase peripheral initialization. The base time is set to 
*                  500usec and the related interrupt is enabled  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Init(void)
{   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	

	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
//	TIM_TimeBaseStructure.TIM_Period = 17999;  
//	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_Period = 17999; //35999;  
	TIM_TimeBaseStructure.TIM_Prescaler = 1; //19;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//定时时间T =（TIM_Period+1）*（TIM_Prescaler+1）/TIMxCLK=(17999+1)*(1+1)/72M = 500 u_second

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM2, ENABLE);	
}

extern rt_uint8_t speed_x;
extern rt_int16_t ENC_MEC_DELTA_ANGLE_BUF_INDEX;
/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{ 
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
					
		/*************************************************************************/
		if (bPID_Position_Sampling_Time_500us != 0)  
		{
			bPID_Position_Sampling_Time_500us --;
		}
		else
		{ 
			bPID_Position_Sampling_Time_500us = PID_POSITION_SAMPLING_TIME;        

				if (State == RUN) 
				{ 
					rt_int16_t hPosition_reference_t = 0;
		
					hPosition_reference_t = hPosition_reference;
					
					Limit_Mec_Position(hPosition_reference_t);    
				}
		}		
		
		/*********************************************************/
		if (bPID_Speed_Sampling_Time_500us != 0 )  
		{
			bPID_Speed_Sampling_Time_500us --;
		}
		else
		{ 
			MCLIB_Mesure_Structure.Mec_Speed = eulerspeedActual*speed_x;
//			MCLIB_Mesure_Structure.Mec_Speed = GET_SPEED_DPS; 
			bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
			ENC_MEC_DELTA_ANGLE_BUF_INDEX = 0;
			if (State == RUN)
			{
				if ((wGlobal_Flags & SPEED_CONTROL) == SPEED_CONTROL)
					FOC_CalcFluxTorqueRef(); 
				else
					FOC_TorqueCtrl();
			}
		}	
	}
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
