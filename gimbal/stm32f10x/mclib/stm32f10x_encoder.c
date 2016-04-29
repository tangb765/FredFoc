/******************************************************************************/
/*                                                                            */
/*                           stm32f10x_encoder.c                              */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include <board.h>
#include "MC_Globals.h"
#include "stm32f10x_MClib.h"
#include "MC_encoder_param.h"
#include "TLE5012.h"
#include "stdlib.h"
#include "app_protocol.h"
#include "MC_PID_regulators.h"
#include "hrt.h"

/* Private typedef -----------------------------------------------------------*/

#define BUF_SIZE 32

rt_int16_t ENC_MEC_ANGLE_BUF[BUF_SIZE];
rt_int16_t ENC_MEC_DELTA_ANGLE_BUF[BUF_SIZE];
rt_int16_t ENC_MEC_ANGLE_BUF_INDEX = 0;
rt_int16_t ENC_MEC_DELTA_ANGLE_BUF_INDEX = 0;
/* Private define ------------------------------------------------------------*/
#define COUNTER_RESET       (rt_uint16_t) ((((rt_int32_t)(ALIGNMENT_ANGLE)*4*ENCODER_PPR/360)\
                                                              -1)/POLE_PAIR_NUM)
#define ICx_FILTER          (rt_uint8_t) 8 // 8<-> 670nsec

#define SPEED_SAMPLING_FREQ (rt_uint16_t)(2000/(PID_SPEED_SAMPLING_TIME+1))

#define TIMx_PRE_EMPTION_PRIORITY 2
#define TIMx_SUB_PRIORITY 0


rt_uint8_t speed_x = 1;

/* Private functions ---------------------------------------------------------*/
rt_int16_t ENC_Calc_Rot_Speed(void);

/* Private variables ---------------------------------------------------------*/
static volatile rt_int16_t hPrevious_angle, hSpeed_Buffer[SPEED_BUFFER_SIZE], hRot_Speed;
//static rt_uint8_t bSpeed_Buffer_Index = 0;
static volatile rt_uint16_t hEncoder_Timer_Overflow; 
static rt_bool_t bIs_First_Measurement = RT_TRUE;
static rt_bool_t bError_Speed_Measurement = RT_FALSE;



rt_int16_t S16_ABS(rt_int16_t temp)
{
	if(temp<0)
		return -temp;
	else
		return temp;
}


/*******************************************************************************
* Function Name  : ENC_Init
* Description    : General Purpose Timer x set-up for encoder speed/position 
*                  sensors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENC_Init(void)
{
	TLE5012_init();
	
	rt_memset(ENC_MEC_ANGLE_BUF, 0, BUF_SIZE);	
	rt_memset(ENC_MEC_DELTA_ANGLE_BUF, 0, BUF_SIZE);
}

rt_int16_t ENC_Get_Electrical_Angle(void) 
{
  rt_int32_t temp;

	temp = (rt_int32_t)(TLE5012_Aligned()) * (rt_int32_t)(2); 
  temp *= POLE_PAIR_NUM;  
	return((rt_int16_t)temp); // rt_int16_t result
}

hrt_abstime time_tps1 = 0;
hrt_abstime time_tps1_last = 0;
hrt_abstime time_tps2 = 0;

rt_uint8_t dfdf = 0;
extern volatile rt_uint8_t bPID_Speed_Sampling_Time_500us;
rt_uint8_t enc_angle_avg_cnt = 1;
rt_int16_t ENC_Get_Mechanical_Angle(void) //16khz
{
  rt_int32_t temp;
  

	
	temp = (rt_int32_t)(TLE5012_Aligned()) * (rt_int32_t)(2);
	//temp = temp & 0xFFFFFFFC;
	ENC_MEC_ANGLE_BUF[ENC_MEC_ANGLE_BUF_INDEX] = (rt_int16_t)temp;
dfdf++;
	
	if(dfdf == 32)
	{
	time_tps1 = hrt_absolute_time();
	time_tps2 = time_tps1 - time_tps1_last;
	time_tps1_last = time_tps1;
	}
	if (!bIs_First_Measurement) 
	{
		ENC_MEC_DELTA_ANGLE_BUF[ENC_MEC_DELTA_ANGLE_BUF_INDEX] = (ENC_MEC_ANGLE_BUF[ENC_MEC_ANGLE_BUF_INDEX] - ENC_MEC_ANGLE_BUF[(ENC_MEC_ANGLE_BUF_INDEX+BUF_SIZE-enc_angle_avg_cnt)%BUF_SIZE])/enc_angle_avg_cnt;
	}

	ENC_MEC_DELTA_ANGLE_BUF_INDEX++;
	ENC_MEC_ANGLE_BUF_INDEX++;
	if(ENC_MEC_ANGLE_BUF_INDEX>=BUF_SIZE)
	{
		bIs_First_Measurement = RT_FALSE;
		ENC_MEC_ANGLE_BUF_INDEX = 0;	
	}	
	
  return	(rt_int16_t)temp; 
}



void ENC_ResetEncoder(void)
{
  //Reset counter
  TLE5012_ResetEncoder();
}


void swap(rt_int16_t *x,rt_int16_t *y)
{
   int temp;
   temp = *x;
   *x = *y;
   *y = temp;
}

rt_int16_t choose_pivot(rt_int16_t i,rt_int16_t j )
{
   return((i+j) /2);
}


void quicksort(rt_int16_t list[],rt_int16_t m,rt_int16_t n)
{
   int key,i,j,k;
   if( m < n)
   {
      k = choose_pivot(m,n);
      swap(&list[m],&list[k]);
      key = list[m];
      i = m+1;
      j = n;
      while(i <= j)
      {
         while((i <= n) && (list[i] <= key))
                i++;
         while((j >= m) && (list[j] > key))
                j--;
         if( i < j)
                swap(&list[i],&list[j]);
      }

      swap(&list[m],&list[j]);

      quicksort(list,m,j-1);
      quicksort(list,j+1,n);
   }
}


rt_int16_t ENC_Calc_Rot_Speed(void)	//1khz
{   
	rt_uint8_t i;
	rt_int16_t tmps[BUF_SIZE];
	signed long long sun_temp=0;
  signed long long temp;
  
//	time_tps1 = hrt_absolute_time();
//	time_tps2 = time_tps1 - time_tps1_last;
//	time_tps1_last = time_tps1;
	
	for(i=0;i<ENC_MEC_DELTA_ANGLE_BUF_INDEX;i++)
	{
		tmps[i] = ENC_MEC_DELTA_ANGLE_BUF[i];
		sun_temp += ENC_MEC_DELTA_ANGLE_BUF[i];
	}
	dfdf = 0;
//	quicksort(tmps,0,BUF_SIZE-1);
//	
//	for(i=4;i<BUF_SIZE-4;i++)
//	{
//		sun_temp += ENC_MEC_DELTA_ANGLE_BUF[i];
//	}
	
//	sun_temp = sun_temp*8/BUF_SIZE+sun_temp;
	
	// speed computation as delta angle * 1/(speed sempling time)
//	temp = (signed long long)(sun_temp * 2000/((bPID_Speed_Sampling_Time_500us+1))); //?¨´?¨¨?¡¤?¦Ì?¨º*¡À??¡¥???¨¨?t????rt_int16_t¡À¨ª¨º? = X    
	temp = (signed long long)(sun_temp * (PWM_FREQ*4/(REP_RATE + 1)/BUF_SIZE)); //?¨´?¨¨?¡¤?¦Ì?¨º*¡À??¡¥???¨¨?t????rt_int16_t¡À¨ª¨º? = X  
	temp *= (45*speed_x);  // 1/360 = 0.00277778 Hz resolution
	temp = temp>>13;	//	temp =(X/65536) *360 ¡ê¡§ ???¨¨/??  ¡ê 
		
	ENC_MEC_DELTA_ANGLE_BUF_INDEX = 0;
  return((rt_int16_t) temp);
}

/*******************************************************************************
* Function Name  : ENC_Clear_Speed_Buffer
* Description    : Clear speed buffer used for average speed calculation  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENC_Clear_Speed_Buffer(void)
{   
  rt_uint32_t i;

  for (i=0;i<SPEED_BUFFER_SIZE;i++)
  {
    hSpeed_Buffer[i] = 0;
  }
  bIs_First_Measurement = RT_TRUE;
}


/*******************************************************************************
* Function Name  : ENC_Get_Mechanical_Speed
* Description    : Export the value of the smoothed motor speed computed in 
*                  ENC_Calc_Average_Speed function  
* Input          : None
* Output         : rt_int16_t
* Return         : Return motor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
rt_int16_t ENC_Get_Mechanical_Speed(void)
{
  return(hRot_Speed);
}


/*******************************************************************************
* Function Name  : ENC_Calc_Average_Speed
* Description    : Compute smoothed motor speed based on last SPEED_BUFFER_SIZE
                   informations and store it variable  
* Input          : None
* Output         : rt_int16_t
* Return         : Return rotor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
rt_int32_t wtemp;
rt_int16_t ENC_Calc_Average_Speed(void)
{   
  
  rt_uint16_t hAbstemp;
//  rt_uint32_t i;
  rt_uint8_t static bError_counter;
  
  wtemp = ENC_Calc_Rot_Speed();
//  wtemp = TLE5012_Calc_Rot_Speed();
  hAbstemp = ( wtemp < 0 ? - wtemp :  wtemp);

/* Checks for speed measurement errors when in RUN State and saturates if 
                                                                    necessary*/  
  if (State == RUN)
  {    
    if(hAbstemp < MOTOR_MIN_SPEED_RPM*speed_x)
    { 
      if (wtemp < 0)
      {
        wtemp = -(rt_int32_t)(MOTOR_MIN_SPEED_RPM*speed_x);
      }
      else
      {
        wtemp = MOTOR_MIN_SPEED_RPM*speed_x;
      }
      bError_counter++;
    }
    else  if (hAbstemp > MOTOR_MAX_SPEED_RPM*speed_x) 
		{
			if (wtemp < 0)
			{
				wtemp = -(rt_int32_t)(MOTOR_MAX_SPEED_RPM*speed_x);
			}
			else
			{
				wtemp = MOTOR_MAX_SPEED_RPM*speed_x;
			}
			bError_counter++;
		}
		else
		{ 
			bError_counter = 0;
		}
  
    if (bError_counter >= MAXIMUM_ERROR_NUMBER)
    {
     bError_Speed_Measurement = RT_TRUE;
    }
    else
    {
     bError_Speed_Measurement = RT_FALSE;
    }
  }
  else
  {
    bError_Speed_Measurement = RT_FALSE;
    bError_counter = 0;
  }
  
/* Compute the average of the read speeds */
  
//  hSpeed_Buffer[bSpeed_Buffer_Index] = (rt_int16_t)wtemp;
//  bSpeed_Buffer_Index++;
//  
//  if (bSpeed_Buffer_Index == SPEED_BUFFER_SIZE) 
//  {
//    bSpeed_Buffer_Index = 0;
//  }

//  wtemp=0;

//  for (i=0;i<SPEED_BUFFER_SIZE;i++)
//    {
//    wtemp += hSpeed_Buffer[i];
//    }
//  wtemp /= SPEED_BUFFER_SIZE;
  
  hRot_Speed = ((rt_int16_t)(wtemp));
		
		return hRot_Speed;
}


/*******************************************************************************
* Function Name  : ENC_ErrorOnFeedback
* Description    : Check for possible errors on speed measurement when State is 
*                  RUN. After MAXIMUM_ERROR_NUMBER consecutive speed measurement
*                  errors, the function return RT_TRUE, else RT_FALSE.
*                  Function return 
* Input          : None
* Output         : rt_int16_t
* Return         : boolean variable
*******************************************************************************/
rt_bool_t ENC_ErrorOnFeedback(void)
{
 return(bError_Speed_Measurement); 
}

rt_int16_t speed_limit_t = 50;
rt_int16_t pos_limit_t = 50;
void Limit_Mec_Position(rt_int16_t pos_set)
{
	rt_int16_t temp;
	
	if(motor_state == ID_DEBUG || motor_state == IQ_DEBUG) return;
	
	temp = eulerActual;
//	temp = Get_Mec_Angle_Value();
	
	if(motor_state == SPD_DEBUG)
	{
		if(abs(temp)>abs(pos_set))
		{
			if(temp >0 && hSpeed_Reference>0)
			{
				hSpeed_Reference = -hSpeed_Reference;
			}
			else if(temp <0 && hSpeed_Reference<0)
			{
				hSpeed_Reference = -hSpeed_Reference;
			}
		}
	}
	else
	{
		hSpeed_Reference = PID_Regulator(pos_set,temp,&configs.pos_pid);
		
		if (hSpeed_Reference>speed_limit_t)
		{
			hSpeed_Reference = speed_limit_t;
		}
		
		if(hSpeed_Reference<-speed_limit_t)
		{
			hSpeed_Reference = -speed_limit_t;
		}
	}
}

#define RAMP_INC_TH   T_ALIGNMENT_PWM_STEPS/(I_ALIGNMENT-256)

void ENC_Start_Up(void)
{
  static rt_uint32_t wTimebase=0;
  
  if ((wGlobal_Flags & FIRST_START) == FIRST_START)
	{
		// First Motor start-up, alignment must be performed
		wTimebase++;

		if(wTimebase <= T_ALIGNMENT_PWM_STEPS)
		{                  
			hFlux_Reference = I_ALIGNMENT;// * wTimebase / T_ALIGNMENT_PWM_STEPS;               
			hTorque_Reference = 0;
			
			Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
			Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
			Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, ALIGNMENT_ANGLE_S16);  

//			/*loads the Torque Regulator output reference voltage Vqs*/   
//			Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
//											Stat_Curr_q_d.qI_Component1, &configs.tqe_pid);

//			
//			/*loads the Flux Regulator output reference voltage Vds*/
//			Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
//												Stat_Curr_q_d.qI_Component2, &configs.flux_pid); 
			
			
			/*loads the Torque Regulator output reference voltage Vqs*/   
			Stat_Volt_q_d.qV_Component1 = 0;

			
			/*loads the Flux Regulator output reference voltage Vds*/
			Stat_Volt_q_d.qV_Component2 = hFlux_Reference; 	

			RevPark_Circle_Limitation();

			/*Performs the Reverse Park transformation,
			i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
			stationary reference frame*/

			Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

			/*Valpha and Vbeta finally drive the power stage*/ 
			CALC_SVPWM(Stat_Volt_alfa_beta);
		}
		else if(wTimebase == 1.2*T_ALIGNMENT_PWM_STEPS)
		{
			ENC_ResetEncoder();  
		}
		else if(wTimebase == 2*T_ALIGNMENT_PWM_STEPS)
		{
			wTimebase = 0;                      
//			Stat_Volt_q_d.qV_Component1 = Stat_Volt_q_d.qV_Component2 = 0;
//			hTorque_Reference = PID_TORQUE_REFERENCE;
//			hFlux_Reference = PID_FLUX_REFERENCE;
			wGlobal_Flags &= ~FIRST_START;   // alignment done only once 
//			//Clear the speed acquisition of the alignment phase
//			ENC_Clear_Speed_Buffer();
			State = STOP;
		}
	}
  else
  {
		if ((configs.Isvalid & 0x8000) && (configs.Isvalid & 0x0080))
			State = RUN;
		else
		{
			output_flag = CLOSE;
			State = STOP;
			hPosition_reference = 0;
			hSpeed_Reference = 0;	
			if(!(configs.Isvalid & 0x8000))
				rt_kprintf("encoder no align.\r\n");
			if(!(configs.Isvalid & 0x0080))
				rt_kprintf("no level mec angle.\r\n");			
		}
  }
} 

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
