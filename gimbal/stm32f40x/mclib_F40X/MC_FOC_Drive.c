/******************************************************************************/
/*                                                                            */
/*                               MC_FOC_Drive.c                               */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_const.h"
#include "MC_FOC_Drive.h"
#include "app_protocol.h"
#include "stdlib.h"
#include "app_config.h"

/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
#define SATURATION_TO_S16(a)    if (a > S16_MAX)              \
                                {                             \
                                  a = S16_MAX;                \
                                }                             \
                                else if (a < -S16_MAX)        \
                                {                             \
                                  a = -S16_MAX;               \
                                }                             \
																
																
/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
static volatile Curr_Components Stat_Curr_q_d_ref;															
static Curr_Components Stat_Curr_q_d_ref_ref;
																

																
/*============================================================================*/
/*                             Function definition                            */
/*============================================================================*/
/*******************************************************************************
* Function Name : FOC_Init
* Description   : The purpose of this function is to initialize to proper values
*                 all the variables related to the field-oriented control
*                 algorithm. To be called once prior to every motor startup.
* Input         : None.
* Output        : None.
* Return        : None.
*******************************************************************************/
void FOC_Init (void)
{
  Stat_Curr_q_d_ref_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref_ref.qI_Component2 = 0;  
  
  Stat_Curr_q_d_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref.qI_Component2 = 0;
}

//#define TEST_ELC_ANGLE
#ifdef TEST_ELC_ANGLE
volatile rt_int16_t Test_elc_angle[16];
static rt_uint8_t Test_cnt = 0;
#endif

//#define Test_Vd_q
#ifdef Test_Vd_q
#define FLT_BUFFERSIZE 2 
rt_int16_t Vd_array[FLT_BUFFERSIZE],Vq_array[FLT_BUFFERSIZE];
rt_int16_t test_Vd_q_cnt=0;
rt_int16_t Vq_wtemp = 0;
rt_int16_t Vd_wtemp = 0;
#endif
/*******************************************************************************
* Function Name : FOC_Model
* Description   : The purpose of this function is to perform PMSM torque and 
*                 flux regulation, implementing the FOC vector algorithm.
* Input         : None.
* Output        : None.
* Return        : None.
*******************************************************************************/
rt_int16_t fsdhfisdhaaa = 0;
void FOC_Model(void)
{ 
  /**********STARTS THE VECTOR CONTROL ************************/  
 
  Stat_Curr_a_b = GET_PHASE_CURRENTS();
  
  Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);
 
//	fsdhfisdhaaa++;
//	Stat_Curr_q_d = Park(	Stat_Curr_alfa_beta,	fsdhfisdhaaa);
	Stat_Curr_q_d = Park(	Stat_Curr_alfa_beta,	(rt_int16_t)(MCLIB_Mesure_Structure.Mec_Angle *	POLE_PAIR_NUM));
	
  /*loads the Torque Regulator output reference voltage Vqs*/   
  Stat_Volt_q_d.qV_Component1 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &configs.tqe_pid);

  
  /*loads the Flux Regulator output reference voltage Vds*/
  Stat_Volt_q_d.qV_Component2 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component2, 
                          Stat_Curr_q_d.qI_Component2, &configs.flux_pid);  												
													
	if(motor_state == IQ_DEBUG)
	{
		/*loads the Flux Regulator output reference voltage Vds*/
		Stat_Volt_q_d.qV_Component2 = 0;  	
	}
	
	if(motor_state == ID_DEBUG)
	{
		/*loads the Torque Regulator output reference voltage Vqs*/   
		Stat_Volt_q_d.qV_Component1 = 0;
	}
	
  //circle limitation
  RevPark_Circle_Limitation();
 
  /*Performs the Reverse Park transformation,
  i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
  stationary reference frame*/

  Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

  /*Valpha and Vbeta finally drive the power stage*/ 
  CALC_SVPWM(Stat_Volt_alfa_beta);
}

extern rt_uint8_t speed_x;
/*******************************************************************************
* Function Name   : FOC_CalcFluxTorqueRef
* Description     : This function provides current components Iqs* and Ids* to be
*                   used as reference values (by the FOC_Model function) when in
*                   speed control mode
* Input           : None.
* Output          : None.
* Return          : None.
*******************************************************************************/
void FOC_CalcFluxTorqueRef(void)
{
	rt_int16_t temp;
	
	configs.spd_pid.Integral_ambit_A = configs.spd_pid.Integral_ambit_A*speed_x;
	configs.spd_pid.Integral_ambit_B = configs.spd_pid.Integral_ambit_B*speed_x;
	Stat_Curr_q_d_ref.qI_Component1 = PID_Regulator(hSpeed_Reference*speed_x,MCLIB_Mesure_Structure.Mec_Speed,&configs.spd_pid);

	Stat_Curr_q_d_ref.qI_Component2 = 0;

  Stat_Curr_q_d_ref_ref = Stat_Curr_q_d_ref;
  
  hTorque_Reference = Stat_Curr_q_d_ref_ref.qI_Component1;
  hFlux_Reference = Stat_Curr_q_d_ref_ref.qI_Component2;  
}

/*******************************************************************************
* Function Name   : FOC_TorqueCntrl
* Description     : This function provides current components Iqs* and Ids* to be
*                   used as reference values (by the FOC_Model function) when in
*                   Torque control mode
* Input           : None.
* Output          : None.
* Return          : None.
*******************************************************************************/
void FOC_TorqueCtrl(void)
{
	rt_int16_t temp;
	
	if(motor_state == ID_DEBUG)
	{
		hFlux_Reference = hSpeed_Reference;
	}

	if(motor_state == IQ_DEBUG)
	{
		temp = Get_Mec_Angle_Value();
		
		if(abs(temp)>abs(hPosition_reference))
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
		
		hTorque_Reference = hSpeed_Reference;
	}
	
	
	Stat_Curr_q_d_ref_ref.qI_Component2 = hFlux_Reference;
  Stat_Curr_q_d_ref_ref.qI_Component1 = hTorque_Reference;
}


/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
