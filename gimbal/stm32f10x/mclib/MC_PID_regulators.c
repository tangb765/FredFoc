/******************************************************************************/
/*                                                                            */
/*                           MC_PID_regulators.c                              */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include "MC_PID_regulators.h"
#include "virtual_eeprom.h"
#include "board.h"
#include "stdlib.h"
#include "MC_PMSM_motor_param.h"


#define S16_MAX    ((rt_int16_t)32767)
#define S16_MIN    ((rt_int16_t)-32768)


#ifdef FeiYu_Board_A
/******************** POSITION PID-CONTROLLER INIT VALUES************************/
#define PID_POSITION_REFERENCE_MEC  (rt_int16_t)0		//1500
#define PID_POSITION_KP_DEFAULT      (rt_int16_t)52
#define PID_POSITION_KI_DEFAULT      (rt_int16_t)0
#define PID_POSITION_KD_DEFAULT      (rt_int16_t)0

/* POSITION PID parameter dividers          */
#define POS_KPDIV ((rt_uint16_t)(512))
#define POS_KIDIV ((rt_uint16_t)(16384))
#define POS_KDDIV ((rt_uint16_t)(512))

/******************** SPEED PID-CONTROLLER INIT VALUES************************/
/* default values for Speed control loop */
#define PID_SPEED_REFERENCE_RPM   (rt_int16_t)10		//1500
#define PID_SPEED_KP_DEFAULT      (rt_int16_t)8000
#define PID_SPEED_KI_DEFAULT      (rt_int16_t)1000
#define PID_SPEED_KD_DEFAULT      (rt_int16_t)0


/* Speed PID parameter dividers          */
#define SP_KPDIV ((rt_uint16_t)(256))
#define SP_KIDIV ((rt_uint16_t)(1024))
#define SP_KDDIV ((rt_uint16_t)(4096))

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/
// With MB459 phase current (A)= (PID_X_REFERENCE * 0.64)/(32767 * Rshunt)
#define PID_TORQUE_REFERENCE   (rt_int16_t)0  //8737 		//(N.b: that's the reference init value in both torque and speed control)
#define PID_TORQUE_KP_DEFAULT  (rt_int16_t)700
#define PID_TORQUE_KI_DEFAULT  (rt_int16_t)1100
#define PID_TORQUE_KD_DEFAULT  (rt_int16_t)0

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (rt_int16_t)0
#define PID_FLUX_KP_DEFAULT  (rt_int16_t)700
#define PID_FLUX_KI_DEFAULT  (rt_int16_t)1100
#define PID_FLUX_KD_DEFAULT  (rt_int16_t)0


// Toruqe/Flux PID  parameter dividers
#define TF_KPDIV ((rt_uint16_t)(4096))
#define TF_KIDIV ((rt_uint16_t)(8192))
#define TF_KDDIV ((rt_uint16_t)(8192))


#define PID_POSITION_AbandonError       5
#define PID_POSITION_hKp_little         60
#define PID_POSITION_Integral_ambit_A   5000
#define PID_POSITION_Integral_ambit_B   182	
	
#define PID_SPEED_AbandonError          0
#define PID_SPEED_hKp_little            0
#define PID_SPEED_Integral_ambit_A      500
#define PID_SPEED_Integral_ambit_B      50
	
#define PID_TORQUE_AbandonError         0
#define PID_TORQUE_hKp_little           0
#define PID_TORQUE_Integral_ambit_A     S16_MAX
#define PID_TORQUE_Integral_ambit_B     S16_MAX

#define PID_FLUX_AbandonError           0
#define PID_FLUX_hKp_little             0
#define PID_FLUX_Integral_ambit_A       S16_MAX
#define PID_FLUX_Integral_ambit_B       S16_MAX
#endif

#ifdef FeiYu_Board_B
/******************** POSITION PID-CONTROLLER INIT VALUES************************/
#define PID_POSITION_REFERENCE_MEC  (rt_int16_t)0		//1500
#define PID_POSITION_KP_DEFAULT      (rt_int16_t)52
#define PID_POSITION_KI_DEFAULT      (rt_int16_t)1
#define PID_POSITION_KD_DEFAULT      (rt_int16_t)0

/* POSITION PID parameter dividers          */
#define POS_KPDIV ((rt_uint16_t)(512))
#define POS_KIDIV ((rt_uint16_t)(16384))
#define POS_KDDIV ((rt_uint16_t)(512))

/******************** SPEED PID-CONTROLLER INIT VALUES************************/
/* default values for Speed control loop */
#define PID_SPEED_REFERENCE_RPM   (rt_int16_t)10		//1500
#define PID_SPEED_KP_DEFAULT      (rt_int16_t)4800
#define PID_SPEED_KI_DEFAULT      (rt_int16_t)1000
#define PID_SPEED_KD_DEFAULT      (rt_int16_t)0

/* Speed PID parameter dividers          */
#define SP_KPDIV ((rt_uint16_t)(64))
#define SP_KIDIV ((rt_uint16_t)(1024))
#define SP_KDDIV ((rt_uint16_t)(4096))

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/
/* default values for Torque control loop */
#define PID_TORQUE_REFERENCE   (rt_int16_t)0  //8737 		//(N.b: that's the reference init value in both torque and speed control)
#define PID_TORQUE_KP_DEFAULT  (rt_int16_t)700
#define PID_TORQUE_KI_DEFAULT  (rt_int16_t)1100
#define PID_TORQUE_KD_DEFAULT  (rt_int16_t)0

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (rt_int16_t)0
#define PID_FLUX_KP_DEFAULT  (rt_int16_t)700
#define PID_FLUX_KI_DEFAULT  (rt_int16_t)1100
#define PID_FLUX_KD_DEFAULT  (rt_int16_t)0

#define TF_KPDIV ((rt_uint16_t)(4096))
#define TF_KIDIV ((rt_uint16_t)(8192))
#define TF_KDDIV ((rt_uint16_t)(8192))

#define PID_POSITION_AbandonError       5
#define PID_POSITION_hKp_little         60
#define PID_POSITION_Integral_ambit_A   5000
#define PID_POSITION_Integral_ambit_B   1820	
	
#define PID_SPEED_AbandonError          0
#define PID_SPEED_hKp_little            0
#define PID_SPEED_Integral_ambit_A      200
#define PID_SPEED_Integral_ambit_B      10
	
#define PID_TORQUE_AbandonError         0
#define PID_TORQUE_hKp_little           0
#define PID_TORQUE_Integral_ambit_A     S16_MAX
#define PID_TORQUE_Integral_ambit_B     S16_MAX

#define PID_FLUX_AbandonError           0
#define PID_FLUX_hKp_little             0
#define PID_FLUX_Integral_ambit_A       S16_MAX
#define PID_FLUX_Integral_ambit_B       S16_MAX
#endif


//#ifdef FeiYu_Board_C
///******************** POSITION PID-CONTROLLER INIT VALUES************************/
//#define PID_POSITION_REFERENCE_MEC  (rt_int16_t)0		//1500
//#define PID_POSITION_KP_DEFAULT      (rt_int16_t)52
//#define PID_POSITION_KI_DEFAULT      (rt_int16_t)1
//#define PID_POSITION_KD_DEFAULT      (rt_int16_t)0

///* POSITION PID parameter dividers          */
//#define POS_KPDIV ((rt_uint16_t)(512))
//#define POS_KIDIV ((rt_uint16_t)(16384))
//#define POS_KDDIV ((rt_uint16_t)(512))

///******************** SPEED PID-CONTROLLER INIT VALUES************************/
///* default values for Speed control loop */
//#define PID_SPEED_REFERENCE_RPM   (rt_int16_t)10		//1500
//#define PID_SPEED_KP_DEFAULT      (rt_int16_t)8000
//#define PID_SPEED_KI_DEFAULT      (rt_int16_t)1000
//#define PID_SPEED_KD_DEFAULT      (rt_int16_t)0

///* Speed PID parameter dividers          */
//#define SP_KPDIV ((rt_uint16_t)(256))
//#define SP_KIDIV ((rt_uint16_t)(1024))
//#define SP_KDDIV ((rt_uint16_t)(4096))

///************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/
///* default values for Torque control loop */
//#define PID_TORQUE_REFERENCE   (rt_int16_t)0  //8737 		//(N.b: that's the reference init value in both torque and speed control)
//#define PID_TORQUE_KP_DEFAULT  (rt_int16_t)700
//#define PID_TORQUE_KI_DEFAULT  (rt_int16_t)1100
//#define PID_TORQUE_KD_DEFAULT  (rt_int16_t)0

///* default values for Flux control loop */
//#define PID_FLUX_REFERENCE   (rt_int16_t)0
//#define PID_FLUX_KP_DEFAULT  (rt_int16_t)700
//#define PID_FLUX_KI_DEFAULT  (rt_int16_t)1100
//#define PID_FLUX_KD_DEFAULT  (rt_int16_t)0

//#define TF_KPDIV ((rt_uint16_t)(4096))
//#define TF_KIDIV ((rt_uint16_t)(8192))
//#define TF_KDDIV ((rt_uint16_t)(8192))

//#define PID_POSITION_AbandonError       5
//#define PID_POSITION_hKp_little         60
//#define PID_POSITION_Integral_ambit_A   5000
//#define PID_POSITION_Integral_ambit_B   182	
//	
//#define PID_SPEED_AbandonError          0
//#define PID_SPEED_hKp_little            0
//#define PID_SPEED_Integral_ambit_A      500
//#define PID_SPEED_Integral_ambit_B      50
//	
//#define PID_TORQUE_AbandonError         0
//#define PID_TORQUE_hKp_little           0
//#define PID_TORQUE_Integral_ambit_A     S16_MAX
//#define PID_TORQUE_Integral_ambit_B     S16_MAX

//#define PID_FLUX_AbandonError           0
//#define PID_FLUX_hKp_little             0
//#define PID_FLUX_Integral_ambit_A       S16_MAX
//#define PID_FLUX_Integral_ambit_B       S16_MAX
//#endif


#ifdef FeiYu_Board_C
/******************** POSITION PID-CONTROLLER INIT VALUES************************/
#define PID_POSITION_REFERENCE_MEC  (rt_int16_t)0		//1500
#define PID_POSITION_KP_DEFAULT      (rt_int16_t)52
#define PID_POSITION_KI_DEFAULT      (rt_int16_t)1
#define PID_POSITION_KD_DEFAULT      (rt_int16_t)0

/* POSITION PID parameter dividers          */
#define POS_KPDIV ((rt_uint16_t)(4096))
#define POS_KIDIV ((rt_uint16_t)(16384))
#define POS_KDDIV ((rt_uint16_t)(512))

/******************** SPEED PID-CONTROLLER INIT VALUES************************/
/* default values for Speed control loop */
#define PID_SPEED_REFERENCE_RPM   (rt_int16_t)10		//1500
#define PID_SPEED_KP_DEFAULT      (rt_int16_t)4800
#define PID_SPEED_KI_DEFAULT      (rt_int16_t)8000
#define PID_SPEED_KD_DEFAULT      (rt_int16_t)0

/* Speed PID parameter dividers          */
#define SP_KPDIV ((rt_uint16_t)(64))
#define SP_KIDIV ((rt_uint16_t)(16384))
#define SP_KDDIV ((rt_uint16_t)(4096))

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/
/* default values for Torque control loop */
#define PID_TORQUE_REFERENCE   (rt_int16_t)0  //8737 		//(N.b: that's the reference init value in both torque and speed control)
#define PID_TORQUE_KP_DEFAULT  (rt_int16_t)700
#define PID_TORQUE_KI_DEFAULT  (rt_int16_t)1100
#define PID_TORQUE_KD_DEFAULT  (rt_int16_t)0

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (rt_int16_t)0
#define PID_FLUX_KP_DEFAULT  (rt_int16_t)700
#define PID_FLUX_KI_DEFAULT  (rt_int16_t)1100
#define PID_FLUX_KD_DEFAULT  (rt_int16_t)0

#define TF_KPDIV ((rt_uint16_t)(4096))
#define TF_KIDIV ((rt_uint16_t)(8192))
#define TF_KDDIV ((rt_uint16_t)(8192))

#define PID_POSITION_AbandonError       5
#define PID_POSITION_hKp_little         60
#define PID_POSITION_Integral_ambit_A   5000
#define PID_POSITION_Integral_ambit_B   182	
	
#define PID_SPEED_AbandonError          0
#define PID_SPEED_hKp_little            0
#define PID_SPEED_Integral_ambit_A      500
#define PID_SPEED_Integral_ambit_B      50
	
#define PID_TORQUE_AbandonError         0
#define PID_TORQUE_hKp_little           0
#define PID_TORQUE_Integral_ambit_A     S16_MAX
#define PID_TORQUE_Integral_ambit_B     S16_MAX

#define PID_FLUX_AbandonError           0
#define PID_FLUX_hKp_little             0
#define PID_FLUX_Integral_ambit_A       S16_MAX
#define PID_FLUX_Integral_ambit_B       S16_MAX
#endif


/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
//#define PID_SPEED_REFERENCE  (rt_uint16_t)(PID_SPEED_REFERENCE_RPM*6)
#define PID_SPEED_REFERENCE  0//50

/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
typedef signed long long s64;
extern rt_uint8_t speed_x;

rt_uint8_t pos_intergral_div = 200;
rt_uint8_t spd_intergral_div = 1;

/*============================================================================*/
/*                             Function definition                            */
/*============================================================================*/
void Init_Config_PID(void)
{
	configs.pos_pid.AbandonError 					= PID_POSITION_AbandonError;
	configs.pos_pid.hKp_little 						= PID_POSITION_hKp_little;
	configs.pos_pid.Integral_ambit_A 			= PID_POSITION_Integral_ambit_A;
	configs.pos_pid.Integral_ambit_B 			= PID_POSITION_Integral_ambit_B;
	configs.pos_pid.hKp_Gain              = PID_POSITION_KP_DEFAULT;
	configs.pos_pid.hKp_Divisor           = POS_KPDIV; 
	configs.pos_pid.hKi_Gain              = PID_POSITION_KI_DEFAULT;
	configs.pos_pid.hKi_Divisor           = POS_KIDIV;  
	configs.pos_pid.hKd_Gain              = PID_POSITION_KD_DEFAULT;
	configs.pos_pid.hKd_Divisor           = POS_KDDIV;
	configs.pos_pid.hLower_Limit_Output   = MOTOR_MIN_SPEED_RPM;
	configs.pos_pid.hUpper_Limit_Output   = MOTOR_MAX_SPEED_RPM;
	configs.pos_pid.wLower_Limit_Integral = MOTOR_MIN_SPEED_RPM * POS_KIDIV /pos_intergral_div;
	configs.pos_pid.wUpper_Limit_Integral = MOTOR_MAX_SPEED_RPM * POS_KIDIV /pos_intergral_div;
	configs.pos_pid.wIntegral             = 0;
	configs.pos_pid.wPreviousError        = 0;
	

	configs.spd_pid.AbandonError 					= PID_SPEED_AbandonError;
	configs.spd_pid.hKp_little 						= PID_SPEED_hKp_little;
	configs.spd_pid.Integral_ambit_A 			= PID_SPEED_Integral_ambit_A;   //uints:dps
	configs.spd_pid.Integral_ambit_B 			= PID_SPEED_Integral_ambit_B;	
	configs.spd_pid.hKp_Gain              = PID_SPEED_KP_DEFAULT;
	configs.spd_pid.hKp_Divisor           = SP_KPDIV;
	configs.spd_pid.hKi_Gain              = PID_SPEED_KI_DEFAULT;
	configs.spd_pid.hKi_Divisor           = SP_KIDIV;  
	configs.spd_pid.hKd_Gain              = PID_SPEED_KD_DEFAULT;
	configs.spd_pid.hKd_Divisor           = SP_KDDIV;
	configs.spd_pid.hLower_Limit_Output   = -IQMAX;
	configs.spd_pid.hUpper_Limit_Output   =  IQMAX;
	configs.spd_pid.wLower_Limit_Integral = -IQMAX * SP_KIDIV /spd_intergral_div;
	configs.spd_pid.wUpper_Limit_Integral =  IQMAX * SP_KIDIV /spd_intergral_div;
	configs.spd_pid.wIntegral             = 0;
	configs.spd_pid.wPreviousError        = 0;
	
	configs.tqe_pid.AbandonError 					= PID_TORQUE_AbandonError;
	configs.tqe_pid.hKp_little 						= PID_TORQUE_hKp_little;
	configs.tqe_pid.Integral_ambit_A 			= PID_TORQUE_Integral_ambit_A;
	configs.tqe_pid.Integral_ambit_B 			= PID_TORQUE_Integral_ambit_B;	
	configs.tqe_pid.hKp_Gain              = PID_TORQUE_KP_DEFAULT;
	configs.tqe_pid.hKp_Divisor           = TF_KPDIV;
	configs.tqe_pid.hKi_Gain              = PID_TORQUE_KI_DEFAULT;
	configs.tqe_pid.hKi_Divisor           = TF_KIDIV;  
	configs.tqe_pid.hKd_Gain              = PID_TORQUE_KD_DEFAULT;
	configs.tqe_pid.hKd_Divisor           = TF_KDDIV;
	configs.tqe_pid.hLower_Limit_Output   = S16_MIN;
	configs.tqe_pid.hUpper_Limit_Output   = S16_MAX;
	configs.tqe_pid.wLower_Limit_Integral = S16_MIN * TF_KIDIV;
	configs.tqe_pid.wUpper_Limit_Integral = S16_MAX * TF_KIDIV;
	configs.tqe_pid.wIntegral             = 0;
	configs.tqe_pid.wPreviousError        = 0;
	
	configs.flux_pid.AbandonError 				 = PID_FLUX_AbandonError;
	configs.flux_pid.hKp_little 					 = PID_FLUX_hKp_little;
	configs.flux_pid.Integral_ambit_A 		 = PID_FLUX_Integral_ambit_A;
	configs.flux_pid.Integral_ambit_B 		 = PID_FLUX_Integral_ambit_B;	
	configs.flux_pid.hKp_Gain              = PID_FLUX_KP_DEFAULT;
	configs.flux_pid.hKp_Divisor           = TF_KPDIV;
	configs.flux_pid.hKi_Gain              = PID_FLUX_KI_DEFAULT;
	configs.flux_pid.hKi_Divisor           = TF_KIDIV;  
	configs.flux_pid.hKd_Gain              = PID_FLUX_KD_DEFAULT;
	configs.flux_pid.hKd_Divisor           = TF_KDDIV;
	configs.flux_pid.hLower_Limit_Output   = S16_MIN;
	configs.flux_pid.hUpper_Limit_Output   = S16_MAX;
	configs.flux_pid.wLower_Limit_Integral = S16_MIN * TF_KIDIV;
	configs.flux_pid.wUpper_Limit_Integral = S16_MAX * TF_KIDIV;
	configs.flux_pid.wIntegral             = 0;
	configs.flux_pid.wPreviousError        = 0;

}


void Init_PID(void)
{


}


void PID_Torque_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.tqe_pid.hKp_Gain    = hGain;
  configs.tqe_pid.hKp_Divisor = hDivisor;
}

void PID_Torque_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.tqe_pid.hKi_Gain    = hGain;
  configs.tqe_pid.hKi_Divisor = hDivisor;  
	configs.tqe_pid.wLower_Limit_Integral = S16_MIN * hDivisor;
	configs.tqe_pid.wUpper_Limit_Integral = S16_MAX * hDivisor;
}

void PID_Torque_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.tqe_pid.hKd_Gain    = hGain;
  configs.tqe_pid.hKd_Divisor = hDivisor;  
}

void PID_Flux_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.flux_pid.hKp_Gain    = hGain;
  configs.flux_pid.hKp_Divisor = hDivisor;  
}

void PID_Flux_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.flux_pid.hKi_Gain    = hGain;
  configs.flux_pid.hKi_Divisor = hDivisor;  
	configs.flux_pid.wLower_Limit_Integral = S16_MIN * hDivisor;
	configs.flux_pid.wUpper_Limit_Integral = S16_MAX * hDivisor;
}

void PID_Flux_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.flux_pid.hKd_Gain    = hGain;
  configs.flux_pid.hKd_Divisor = hDivisor;  
}

void PID_Speed_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.spd_pid.hKp_Gain    = hGain;
  configs.spd_pid.hKp_Divisor = hDivisor;  
}


void PID_Speed_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.spd_pid.hKi_Gain    = hGain;
  configs.spd_pid.hKi_Divisor = hDivisor;  
	configs.spd_pid.wLower_Limit_Integral = -IQMAX * hDivisor/spd_intergral_div;
	configs.spd_pid.wUpper_Limit_Integral =  IQMAX * hDivisor/spd_intergral_div;
}

void PID_Speed_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.spd_pid.hKd_Gain    = hGain;
  configs.spd_pid.hKd_Divisor = hDivisor;  
}

void PID_Position_Kp_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.pos_pid.hKp_Gain    = hGain;
  configs.pos_pid.hKp_Divisor = hDivisor;  
}


void PID_Position_Ki_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.pos_pid.hKi_Gain    = hGain;
  configs.pos_pid.hKi_Divisor = hDivisor;  
	configs.pos_pid.wLower_Limit_Integral = MOTOR_MIN_SPEED_RPM * hDivisor/pos_intergral_div;
	configs.pos_pid.wUpper_Limit_Integral = MOTOR_MAX_SPEED_RPM * hDivisor/pos_intergral_div;
}

void PID_Position_Kd_update(rt_int16_t hGain,rt_uint16_t hDivisor)
{
  configs.pos_pid.hKd_Gain    = hGain;
  configs.pos_pid.hKd_Divisor = hDivisor;  
}

/*******************************************************************************
* Function Name  : PID_Regulator
* Description    : Compute the PI(D) output for a PI(D) regulation.
* Input          : Pointer to the PID settings (*PID_Flux)
                   Speed in rt_int16_t format
* Output         : rt_int16_t
* Return         : None
*******************************************************************************/


rt_int16_t PID_Regulator(rt_int16_t hReference, rt_int16_t hPresentFeedback, struct pid_struct_t *PID_Struct)
{
  rt_int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
	rt_uint32_t abs_wError=0;
  s64 dwAux; 
	static rt_int32_t Integral_enable_div;
#ifdef DIFFERENTIAL_TERM_ENABLED    
  rt_int32_t wDifferential_Term;
#endif    
  // error computation
  wError= (rt_int32_t)(hReference - hPresentFeedback);
	
	abs_wError = abs(wError);
	
	if(abs_wError<PID_Struct->AbandonError)
	{
		wError = 0;
	}
  // Proportional term computation
  wProportional_Term = (rt_int32_t)PID_Struct->hKp_Gain * wError;
	if(abs_wError<PID_Struct->Integral_ambit_A)
	{
		wProportional_Term += (rt_int32_t)PID_Struct->hKp_little * wError * (rt_int32_t)1000 /(rt_int32_t)(500+abs_wError);
	}
  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
		
		if(abs_wError<(PID_Struct->Integral_ambit_A + PID_Struct->Integral_ambit_B))
		{
			
			if(abs_wError<PID_Struct->Integral_ambit_B)
			{
				wIntegral_Term = (rt_int32_t)PID_Struct->hKi_Gain * wError ;
			}
			else
			{
				if(wError>0)
				{
					Integral_enable_div = (rt_int32_t)PID_Struct->Integral_ambit_A + (rt_int32_t)PID_Struct->Integral_ambit_B - wError;
				}
				else
				{
					Integral_enable_div = (rt_int32_t)PID_Struct->Integral_ambit_A + (rt_int32_t)PID_Struct->Integral_ambit_B + wError;
				}
				wIntegral_Term = (PID_Struct->hKi_Gain * wError * Integral_enable_div)/(PID_Struct->Integral_ambit_A + PID_Struct->Integral_ambit_B);
			}
		}
		else
		{
			wIntegral_Term = 0;
		}
    
    dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);
    
    if (dwAux > PID_Struct->wUpper_Limit_Integral)
    {
      PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
    }
    else if (dwAux < PID_Struct->wLower_Limit_Integral)
          { 
            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
          }
          else
          {
						PID_Struct->wIntegral = (rt_int32_t)(dwAux);
          }
  }
  // Differential term computation
#ifdef DIFFERENTIAL_TERM_ENABLED
  {
  rt_int32_t wtemp;
  
  wtemp = wError - PID_Struct->wPreviousError;
  wDifferential_Term = PID_Struct->hKd_Gain * wtemp;
  PID_Struct->wPreviousError = wError;    // store value 
  }
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
                wDifferential_Term/PID_Struct->hKd_Divisor); 

#else  
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
#endif
  
    if (houtput_32 >= PID_Struct->hUpper_Limit_Output)
      {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
      }
    else if (houtput_32 < PID_Struct->hLower_Limit_Output)
      {
      return(PID_Struct->hLower_Limit_Output);
      }
    else 
      {
				if(abs_wError<PID_Struct->AbandonError)
				{
					houtput_32 = 0;
					PID_Struct->wIntegral = 0;
				}					
				
        return((rt_int16_t)(houtput_32)); 		
      }
}		 

//rt_int16_t PID_Regulator(rt_int16_t hReference, rt_int16_t hPresentFeedback, struct pid_struct_t *PID_Struct)
//{
//  rt_int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
//	rt_uint32_t abs_wError=0;
//  s64 dwAux; 
//	static rt_int32_t Integral_enable_div;
//#ifdef DIFFERENTIAL_TERM_ENABLED    
//  rt_int32_t wDifferential_Term;
//#endif    
//	
//  // error computation
//  wError= (rt_int32_t)(hReference - hPresentFeedback);
//	
//	abs_wError = abs(wError);
//	
//	if(abs_wError<PID_Struct->AbandonError)
//	{
//		wError = 0;
//	}
//  // Proportional term computation
//  wProportional_Term = (rt_int32_t)PID_Struct->hKp_Gain * wError;
//	if(abs_wError<PID_Struct->Integral_ambit_A)
//	{
//		wProportional_Term += (rt_int32_t)PID_Struct->hKp_little * wError * (rt_int32_t)1000 /(rt_int32_t)(500+abs_wError);
//	}
//  // Integral term computation
//  if (PID_Struct->hKi_Gain == 0)
//  {
//    PID_Struct->wIntegral = 0;
//  }
//  else
//  { 
//		wIntegral_Term = PID_Struct->hKi_Gain * wError;
//		
//		if(abs_wError<PID_Struct->Integral_ambit_A)
//		{
//			if(abs_wError>PID_Struct->Integral_ambit_B)
//			{
//				Integral_enable_div = abs_wError - PID_Struct->Integral_ambit_B;
//				wIntegral_Term = (s64)(PID_Struct->hKi_Gain * wError * Integral_enable_div)/(PID_Struct->Integral_ambit_A - PID_Struct->Integral_ambit_B);
//			}
//		}
//		else
//		{
//			wIntegral_Term = 0;
//		}

////		if(abs_wError<PID_Struct->Integral_ambit_A)
////		{
////			if(abs_wError<PID_Struct->Integral_ambit_B)
////			{
////				wIntegral_Term = (rt_int32_t)PID_Struct->hKi_Gain * wError;
////			}
////			else
////			{
////				Integral_enable_div = abs_wError - PID_Struct->Integral_ambit_B;
////				wIntegral_Term = (s64)(PID_Struct->hKi_Gain * wError * Integral_enable_div)/(PID_Struct->Integral_ambit_A - PID_Struct->Integral_ambit_B);
////			}
////		}
////		else
////		{
////			wIntegral_Term = 0;
////		}
//    
//    dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);
//    
//    if (dwAux > PID_Struct->wUpper_Limit_Integral)
//    {
//      PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
//    }
//    else if (dwAux < PID_Struct->wLower_Limit_Integral)
//          { 
//            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
//          }
//          else
//          {
//						PID_Struct->wIntegral = (rt_int32_t)(dwAux);
//          }
//  }
//  // Differential term computation
//#ifdef DIFFERENTIAL_TERM_ENABLED
//  {
//  rt_int32_t wtemp;
//  
//  wtemp = wError - PID_Struct->wPreviousError;
//  wDifferential_Term = PID_Struct->hKd_Gain * wtemp;
//  PID_Struct->wPreviousError = wError;    // store value 
//  }
//  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
//                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
//                wDifferential_Term/PID_Struct->hKd_Divisor); 

//#else  
//  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
//                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
//#endif
//  
//    if (houtput_32 >= PID_Struct->hUpper_Limit_Output)
//      {
//      return(PID_Struct->hUpper_Limit_Output);		  			 	
//      }
//    else if (houtput_32 < PID_Struct->hLower_Limit_Output)
//      {
//      return(PID_Struct->hLower_Limit_Output);
//      }
//    else 
//      {
//				if(abs_wError<PID_Struct->AbandonError)
//				{
//					houtput_32 = 0;
//					PID_Struct->wIntegral = 0;
//				}					
//				
//        return((rt_int16_t)(houtput_32)); 		
//      }
//}


/******************** (C) COPYRIGHT 2008 STMicroelectronics *******************/


