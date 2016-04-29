/******************************************************************************/
/*                                                                            */
/*                                 mian.c                                     */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include <board.h>
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include <rtthread.h>
#include "stm32f10x_encoder.h"
#include "TLE5012.h"
#include "hrt.h"

/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
#define BRAKE_HYSTERESIS (rt_uint16_t)((OVERVOLTAGE_THRESHOLD/16)*15)

/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
unsigned char Res_f=0;

int dsadasd,dsadasd1=0;
extern rt_int16_t Mart_test_angle;
//extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);

void Mclib_API_init(void)
{
	ENC_Init();
	
  SVPWM_3ShuntInit();

  TB_Init();

//#ifdef BRAKE_RESISTOR
//    MCL_Brake_Init();
//#endif

	/* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  
  // Init Bus voltage and Temperature average  
  MCL_Init_Arrays();
	
  Res_f=1;

}


void Mclib_API_thread_entry(void* parameter)
{

    /* Main PWM Output Enable */
    TIM_CtrlPWMOutputs(TIM1,ENABLE);


    // Enable the Adv Current Reading during Run state
    SVPWM_3ShuntAdvCurrentReading(ENABLE);
		
    while (1)
    {
			
				switch (State)
				{
					case IDLE:    // Idle state   
					break;
					
					case INIT:
						MCL_Init();
						delay_ms(15);
						State = START;
					break;
						
					case START: 
					break;
							
					case RUN:   // motor running       
						if(ENC_ErrorOnFeedback() == RT_TRUE)
						{
							//LEDToggle(LED1);
							MCL_SetFault(SPEED_FEEDBACK);
						}
						
					break;  
					
					case STOP:    // motor stopped
							// shutdown power         
							/* Main PWM Output Disable */
//							TIM_CtrlPWMOutputs(TIM1, DISABLE);
							F40X_TurnOfflowsides();
							
							State = WAIT;
										 
							SVPWM_3ShuntAdvCurrentReading(DISABLE);

							SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);
															
							delay_ms(10);
					break;
					
					case WAIT:    // wait state
							if(ENC_Get_Mechanical_Speed() ==0)             
							{              
								State = IDLE;              
							}       
						break;
				
					case FAULT:                   
							if (MCL_ClearFault() == RT_TRUE)
							{ 
								State = IDLE;
							}
						break;
				
					default:        
						break;
				}
	
			rt_thread_delay( RT_TICK_PER_SECOND/250 );					
		}
}


int MClib_application_init(void)
{
		Mclib_API_init();

   return 0;
}


/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_IRQHandler(void)
{
  if((ADC1->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
  {
    //It clear JEOC flag
    ADC1->SR &= ~(rt_uint32_t)(ADC_FLAG_JEOC | ADC_FLAG_JSTRT);
    MCLIB_Mesure_Structure.Mec_Angle = GET_MECHANICAL_ANGLE;
//    if (SVPWMEOCEvent())
		if (State != IDLE)
    {   
//      MCL_Calc_BusVolt();
			
      switch (State)
      {
          case RUN:          
            FOC_Model();       
          break;       
    
          case START: 
						ENC_Start_Up();	
          break; 
    
          default:
          break;
      }
			
      #ifdef BRAKE_RESISTOR
        if((wGlobal_Flags & BRAKE_ON) == BRAKE_ON)
        {
          rt_uint16_t aux;
					
          aux = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
        
          if (aux < BRAKE_HYSTERESIS)
          {
           wGlobal_Flags &= ~BRAKE_ON;
           MCL_Set_Brake_Off();
          }
        }
      #endif      
    }
  }
  else 
  {  
    if(ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
    {
#ifdef BRAKE_RESISTOR
      //Analog watchdog interrupt has been generated 
     MCL_Set_Brake_On(); 
     wGlobal_Flags |= BRAKE_ON;
#else
     if(MCL_Chk_BusVolt()==OVER_VOLT)  // ·ÀÖ¹¸ÉÈÅ
        MCL_SetFault(OVER_VOLTAGE);
#endif

     ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
    }    
  }
	
}



/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
  if(Res_f==1)
  MCL_SetFault(OVER_CURRENT);
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
}

/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
  // Clear Update Flag
  TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(rt_uint8_t* file, rt_uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    //printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
