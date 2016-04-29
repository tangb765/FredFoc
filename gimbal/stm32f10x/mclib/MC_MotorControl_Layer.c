/******************************************************************************/
/*                                                                            */
/*                        MC_MotorControl_Layer.c                             */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "app_config.h"
#include "hrt.h"

/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
#define BRK_GPIO GPIOE
#define BRK_PIN GPIO_Pin_15

#define FAULT_STATE_MIN_PERMANENCY 600 //0.5msec unit

#define BUS_AV_ARRAY_SIZE  (rt_uint8_t)64  //number of averaged acquisitions
#define T_AV_ARRAY_SIZE  (rt_uint16_t)2048  //number of averaged acquisitions

#define BUSV_CONVERSION (rt_uint16_t) (3.32/(BUS_ADC_CONV_RATIO)) 
#define TEMP_CONVERSION (rt_uint8_t)  195

#define VOLT_ARRAY_INIT (rt_uint16_t)(UNDERVOLTAGE_THRESHOLD+ OVERVOLTAGE_THRESHOLD)/2
#define TEMP_ARRAY_INIT (rt_uint16_t)0

#define BRAKE_GPIO_PORT       GPIOD
#define BRAKE_GPIO_PIN        GPIO_Pin_13

#define NTC_THRESHOLD (rt_uint16_t) ((32768*(NTC_THRESHOLD_C - 14))/TEMP_CONVERSION)
#define NTC_HYSTERIS  (rt_uint16_t) ((32768*(NTC_THRESHOLD_C - NTC_HYSTERIS_C - 14))\
                                                               /TEMP_CONVERSION)

/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
void MCL_Reset_PID_IntegralTerms(void);
/* Private variables ---------------------------------------------------------*/

static rt_int16_t h_BusV_Average;
static rt_uint32_t w_Temp_Average;

rt_uint16_t h_ADCBusvolt;
rt_uint16_t h_ADCTemp;

/*============================================================================*/
/*                             Function definition                            */
/*============================================================================*/
/*******************************************************************************
* Function Name  : MCL_Init
* Description    : This function implements the motor control initialization to 
*                  be performed at each motor start-up 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init(void)
{
// reset PID's integral values
    MCL_Reset_PID_IntegralTerms();
    FOC_Init();

    ENC_Clear_Speed_Buffer();
                 
    SVPWM_3ShuntCurrentReadingCalibration();
      
    Stat_Volt_alfa_beta.qV_Component1 = 0;
    Stat_Volt_alfa_beta.qV_Component2 = 0;             
    CALC_SVPWM(Stat_Volt_alfa_beta);	//给定初始化alfa_beta电压为零，换算成三相占空比
    //hTorque_Reference = PID_TORQUE_REFERENCE;  //给定初始转矩 
    
    /* Main PWM Output Enable */
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
    //It generates for 2 msec a 50% duty cycle on the three phases to load Boot 
    //capacitance of high side drivers
		delay_ms(2);

    // Enable the Adv Current Reading during Run state
    SVPWM_3ShuntAdvCurrentReading(ENABLE);

}


/*******************************************************************************
* Function Name  : MCL_Init_Arrays
* Description    : This function initializes array to avoid erroneous Fault 
*                  detection after a reswt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init_Arrays(void)
{   
    w_Temp_Average = TEMP_ARRAY_INIT;
    h_BusV_Average = VOLT_ARRAY_INIT;   
}


/*******************************************************************************
* Function Name  : MCL_ChkPowerStage
* Description    : This function check for power stage working conditions
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_ChkPowerStage(void) 
{
    //  check over temperature of power stage
    if (MCL_Chk_OverTemp() == RT_TRUE) 
    {
      MCL_SetFault(OVERHEAT);
    }   
    //  check bus under voltage 
    if (MCL_Chk_BusVolt() == UNDER_VOLT) 
    {
      MCL_SetFault(UNDER_VOLTAGE);
    }
    // bus over voltage is detected by analog watchdog
}

/*******************************************************************************
* Function Name  : MCL_SetFault() 
* Description    : This function manage faults occurences
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_SetFault(rt_uint16_t hFault_type)
{
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  wGlobal_Flags |= hFault_type;
  State = FAULT;
	
  // It is required to disable AdvCurrentReading in IDLE to sample DC 
  // Bus Value
  SVPWM_3ShuntAdvCurrentReading(DISABLE);
	
  delay_ms(1);
}

/*******************************************************************************
* Function Name  : MCL_ClearFault() 
* Description    : This function check if the fault source is over. In case it 
*                  is, it clears the related flag and return RT_TRUE. Otherwise it 
*                  returns RT_FALSE
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
rt_bool_t MCL_ClearFault(void)
{     
	if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)   
	{               
		if(MCL_Chk_OverTemp()== RT_FALSE)
		{
			wGlobal_Flags &= ~OVERHEAT;
		}     
	}
	
	if ((wGlobal_Flags & OVER_VOLTAGE) == OVER_VOLTAGE)   
	{            
		if(MCL_Chk_BusVolt()== NO_FAULT)
		{
			wGlobal_Flags &= ~OVER_VOLTAGE;
		} 
	}
	
	if ((wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)   
	{            
		if(MCL_Chk_BusVolt()== NO_FAULT)
		{
			wGlobal_Flags &= ~UNDER_VOLTAGE;
		} 
	}
	
	if ((wGlobal_Flags & OVER_CURRENT) == OVER_CURRENT)
	{
		// high level detected on emergency pin?              
		//It checks for a low level on MCES before re-enable PWM 
		//peripheral
		if (GPIO_ReadInputDataBit(BRK_GPIO, BRK_PIN))
		{            
			wGlobal_Flags &= ~OVER_CURRENT;
		}
	}

	if ((wGlobal_Flags & START_UP_FAILURE) == START_UP_FAILURE )
	{
			wGlobal_Flags &= ~START_UP_FAILURE;
	} 
	
	if ((wGlobal_Flags & SPEED_FEEDBACK) == SPEED_FEEDBACK )
	{
			wGlobal_Flags &= ~SPEED_FEEDBACK;
	} 

	if ( (wGlobal_Flags & (OVER_CURRENT | OVERHEAT | UNDER_VOLTAGE | 
										 SPEED_FEEDBACK | START_UP_FAILURE | OVER_VOLTAGE)) == 0 )       
	{ 
		return(RT_TRUE); 
	} 
	else
	{
		return(RT_FALSE);
	}
}

/*******************************************************************************
* Function Name  : MCL_Chk_OverTemp
* Description    : Return RT_TRUE if the voltage on the thermal resistor connected 
*                  to channel AIN3 has reached the threshold level or if the           
*                  voltage has not yet reached back the threshold level minus  
*                  the hysteresis value after an overheat detection.
* Input          : None
* Output         : Boolean
* Return         : None
*******************************************************************************/
rt_bool_t MCL_Chk_OverTemp(void)
{
  rt_bool_t bStatus;
   
  w_Temp_Average = ((T_AV_ARRAY_SIZE-1)*w_Temp_Average + h_ADCTemp)
                                                              /T_AV_ARRAY_SIZE;
  
  if (w_Temp_Average >= NTC_THRESHOLD)    
  {
    bStatus = RT_TRUE;
  }
  else if (w_Temp_Average >= (NTC_HYSTERIS) ) 
    {
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)
      {
        bStatus = RT_TRUE;       
      }
    else
      {
        bStatus = RT_FALSE;
      }
    }
  else 
    {
      bStatus = RT_FALSE;
    }

		bStatus = bStatus;
  //return(bStatus);
  return(RT_FALSE);
  //温度过热，暂时屏掉
}

/*******************************************************************************
* Function Name  : MCL_Calc_BusVolt
* Description    : It measures the Bus Voltage
* Input          : None
* Output         : Bus voltage
* Return         : None
*******************************************************************************/
void MCL_Calc_BusVolt(void)
{
 h_BusV_Average = ((BUS_AV_ARRAY_SIZE-1)*h_BusV_Average + h_ADCBusvolt)
                                                             /BUS_AV_ARRAY_SIZE;
}

/*******************************************************************************
* Function Name  : MCL_Chk_BusVolt 
* Description    : Check for Bus Over Voltage
* Input          : None
* Output         : Boolean
* Return         : None
*******************************************************************************/
BusV_t MCL_Chk_BusVolt(void)
{
  BusV_t baux;
  if (h_BusV_Average > OVERVOLTAGE_THRESHOLD)    
  {
    baux = OVER_VOLT;
  }
  else if (h_BusV_Average < UNDERVOLTAGE_THRESHOLD)    
  {
    baux = UNDER_VOLT;
  }
  else 
  {
    baux = NO_FAULT; 
  }
  return ((BusV_t)baux);
}

/*******************************************************************************
* Function Name  : MCL_Get_BusVolt
* Description    : Get bus voltage in rt_int16_t
* Input          : None
* Output         : None
* Return         : Bus voltage in rt_int16_t unit
*******************************************************************************/
rt_int16_t MCL_Get_BusVolt(void)
{
  return (11*(BUS_ADC_CONV_RATIO*32768/3.3));//(h_BusV_Average);
}

/*******************************************************************************
* Function Name  : MCL_Compute_BusVolt
* Description    : Compute bus voltage in volt
* Input          : None
* Output         : Bus voltage in Volt unit
* Return         : None
*******************************************************************************/
rt_uint16_t MCL_Compute_BusVolt(void)
{
  return ((rt_uint16_t)((h_BusV_Average * BUSV_CONVERSION)/32768));
}

/*******************************************************************************
* Function Name  : MCL_Compute_Temp
* Description    : Compute temperature in Celsius degrees
* Input          : None
* Output         : temperature in Celsius degrees
* Return         : None
*******************************************************************************/
rt_uint8_t MCL_Compute_Temp(void)
{
  return ((rt_uint8_t)((w_Temp_Average * TEMP_CONVERSION)/32768+14));
}      

/*******************************************************************************
* Function Name  : MCL_Reset_PID_IntegralTerms
* Description    : Resets flux, torque and speed PID Integral Terms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Reset_PID_IntegralTerms(void)
{
	configs.pos_pid.wIntegral=0;
	configs.spd_pid.wIntegral=0;
	configs.tqe_pid.wIntegral=0;
	configs.flux_pid.wIntegral=0;
}


#ifdef BRAKE_RESISTOR
/*******************************************************************************
* Function Name  : MCL_Brake_Init
* Description    : Initialize the GPIO driving the switch for resitive brake 
*                  implementation  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Brake_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOD clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_DeInit(BRAKE_GPIO_PORT);
  GPIO_StructInit(&GPIO_InitStructure);
                  
  /* Configure PD.13 as Output push-pull for break feature */
  GPIO_InitStructure.GPIO_Pin = BRAKE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BRAKE_GPIO_PORT, &GPIO_InitStructure);     
}

/*******************************************************************************
* Function Name  : MCL_Set_Brake_On
* Description    : Switch on brake (set the related GPIO pin)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Set_Brake_On(void)
{  
 GPIO_SetBits(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : MCL_Set_Brake_Off
* Description    : Switch off brake (reset the related GPIO pin)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Set_Brake_Off(void)
{  
 GPIO_ResetBits(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN);
}

#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
