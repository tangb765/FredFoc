/******************************************************************************/
/*                                                                            */
/*                         stm32f10x_svpwm_2shunt.c                           */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include "stm32f4xx.h"
#include "stm32f10x_svpwm_3shunt.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//#define LS_PWM_TIMER
#define LS_GPIO


#define NB_CONVERSIONS 16

#define SQRT_3		1.732051
#define T		    (PWM_PERIOD * 4)
#define T_SQRT3     (rt_uint16_t)(T * SQRT_3)

#define SECTOR_1	(rt_uint32_t)1
#define SECTOR_2	(rt_uint32_t)2
#define SECTOR_3	(rt_uint32_t)3
#define SECTOR_4	(rt_uint32_t)4
#define SECTOR_5	(rt_uint32_t)5
#define SECTOR_6	(rt_uint32_t)6

#define PHASE_A_ADC_CHANNEL     ADC_Channel_0
#define PHASE_B_ADC_CHANNEL     ADC_Channel_1
#define PHASE_C_ADC_CHANNEL     ADC_Channel_2


// Settings for current sampling only
#define PHASE_A_MSK       (rt_uint32_t)((rt_uint32_t)(PHASE_A_ADC_CHANNEL) << 15)
#define PHASE_B_MSK       (rt_uint32_t)((rt_uint32_t)(PHASE_B_ADC_CHANNEL) << 15)
#define PHASE_C_MSK       (rt_uint32_t)((rt_uint32_t)(PHASE_C_ADC_CHANNEL) << 15)

// Settings for current sampling only
#define TEMP_FDBK_MSK     (rt_uint32_t)(0)
#define BUS_VOLT_FDBK_MSK (rt_uint32_t)(0)

// Settings for current sampling only
#define SEQUENCE_LENGHT    0x00000000

#define ADC_PRE_EMPTION_PRIORITY 1
#define ADC_SUB_PRIORITY 0

#define BRK_PRE_EMPTION_PRIORITY 0
#define BRK_SUB_PRIORITY 0

#define TIM1_UP_PRE_EMPTION_PRIORITY 1
#define TIM1_UP_SUB_PRIORITY 0

#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset

#define PWM2_MODE 0
#define PWM1_MODE 1

/* Bit Banding */
#define ADC1_CR2_SWSTART_BB 0x42240178u

#define TIMxCCER_MASK              ((rt_uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((rt_uint16_t)  0x555u)

#define TIMx_CC4E_BIT              ((rt_uint16_t)  0x1000u) 

#define CONV_STARTED               ((rt_uint32_t) (0x8))
#define CONV_FINISHED              ((rt_uint32_t) (0xC))
#define FLAGS_CLEARED              ((rt_uint32_t) (0x0))
#define ADC_SR_MASK                ((rt_uint32_t) (0xC))

#define ADC_RIGHT_ALIGNMENT 3u


#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)
#define AUX_CR2_JEXTTRIG 0x001E1901u

#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define CCMR2_CH4_PWM2    0x7000u

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
rt_uint8_t  bSector;  

rt_uint16_t hPhaseAOffset;
rt_uint16_t hPhaseBOffset;
//rt_uint16_t hPhaseCOffset;


rt_uint16_t ADCTriggerSet;
rt_uint16_t ADCTriggerUnSet;

rt_uint8_t PWM4Direction=PWM2_MODE;


/* Private function prototypes -----------------------------------------------*/
void SVPWM_InjectedConvConfig(void);
Curr_Components Get_Current_offset(void);


static void F40X_GetPhaseCurrent(void);
void F40X_TurnOnlowsides(void);
void F40X_TurnOfflowsides(void);
void F40X_SwitchOnPWM(void);
void F40X_SwitchOffPWM(void);


static void F40x_svpwm_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | 
											 RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | 
												 RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | 
													 RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG |
														 RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI, ENABLE);
	  
  GPIO_StructInit(&GPIO_InitStructure);
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  //Iu_adc
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinLockConfig(GPIOA, GPIO_Pin_0);
  
  /****** Configure phase B ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //Iv_adc
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinLockConfig(GPIOA, GPIO_Pin_1);
  
  /****** Configure phase C ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  //Vbat_adc
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinLockConfig(GPIOA, GPIO_Pin_2);	

  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
   
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinLockConfig(GPIOA, GPIO_Pin_8);
  GPIO_PinLockConfig(GPIOA, GPIO_Pin_9);
  GPIO_PinLockConfig(GPIOA, GPIO_Pin_10);

 /****** Configure TIMx Channel 1N, 2N and 3N Outputs, if enabled ******/ 
	#ifdef LS_PWM_TIMER
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinLockConfig(GPIOB, GPIO_Pin_13);
  GPIO_PinLockConfig(GPIOB, GPIO_Pin_14);
  GPIO_PinLockConfig(GPIOB, GPIO_Pin_15);
	#endif

	#ifdef LS_GPIO
	/*******************************L6230 lowside control by one GPIO**************************************/
	// /**** Only "active high" polarity is supported *****/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinLockConfig(GPIOB, GPIO_Pin_9);
	
	F40X_TurnOfflowsides();
	#endif	
	
  /* BKIN, if enabled */
  #ifdef BKIN_Mode 	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //上面改成GPIO_Mode_OUT，这里改回 GPIO_Mode_AF	
  /****** Configure TIMx BKIN input, if enabled ******/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_PinLockConfig(GPIOB, GPIO_Pin_12);	
	#endif
}




static void F40X_TIM1Init(void)
{
  TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
  TIM_OCInitTypeDef TIM1_OCInitStructure;
  TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;	
	
	  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
  /* Time Base configuration */
  TIM1_TimeBaseStructure.TIM_Prescaler = (rt_uint16_t)PWM_PRSC;
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;
  TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
  
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIM1_OCInitStructure);  
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_Pulse = (rt_uint32_t)(PWM_PERIOD)/2u; /* dummy value */
 
  /* Channel 1 */
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;    
  #ifdef LS_PWM_TIMER
  {
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; 
    TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;     
  }    
  #else
  {
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
	#endif
  TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
  
  
  /* Channel 2 */
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;    
  #ifdef LS_PWM_TIMER
  {
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; 
  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;         
  }
	#endif
  TIM_OC2Init(TIM1, &TIM1_OCInitStructure); 
  
  
  /* Channel 3 */
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;    
  #ifdef LS_PWM_TIMER
  {
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; 
  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;         
  }
	#endif
  TIM_OC3Init(TIM1, &TIM1_OCInitStructure);   
  
    /* Channel 4 */
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;     	
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 
  TIM1_OCInitStructure.TIM_Pulse = (uint32_t)(PWM_PERIOD) -10u;
  TIM_OC4Init(TIM1, &TIM1_OCInitStructure); 
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 
  
	

  TIM_BDTRStructInit(&TIM1_BDTRInitStructure);
  /* Dead Time */
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME/2u;
	
  /* BKIN, if enabled */
  #ifdef BKIN_Mode  
  {
    TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
  }  
	#endif
  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);
 
	
  // Resynch to have the Update evend during Undeflow
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);	
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
  TIM_ITConfig(TIM1, TIM_IT_Break,DISABLE);
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  // Clear Update Flag
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
  TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);	
		

  TIM_SetCounter(TIM1, (uint32_t)((PWM_PERIOD)/2u)-1u);
	
  /* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);	
}


#ifdef LS_GPIO
void F40X_TurnOnlowsides(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
}
void F40X_TurnOfflowsides(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
}
#endif

static void Inject_adc_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
 /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 	

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* ADC Init */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfConversion = 1u;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_InjectedDiscModeCmd(ADC1, ENABLE);
  ADC_InjectedDiscModeCmd(ADC2, ENABLE);
	
/* It is used only to configure the sampling time to the corresponding channel*/
  ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL, 1u,
                                                ADC_SampleTime_15Cycles);

  ADC_InjectedChannelConfig(ADC2, PHASE_B_ADC_CHANNEL, 1u,
                                                ADC_SampleTime_15Cycles);

//	ADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4) */
//	ADCTriggerSet   = ADCTriggerUnSet | 0x00100000u; /* JEXTEN = 01b (Enable), JEXTSEL = 0000b (TIM1_CC4) */
//	ADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4) */
//	ADCTriggerSet   = ADCTriggerUnSet | 0x00110000u; /* JEXTEN = 01b (Enable), JEXTSEL = 0000b (TIM1_TRGO) */
  
  /* ADC1 Injected conversions end interrupt enabling */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	 
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);  	
}

void SVPWM_3ShuntInit(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	F40x_svpwm_GPIO_init();	
	F40X_TIM1Init();
//	TIM_CtrlPWMOutputs(TIM1, DISABLE);
		 
	/* Enable the TIM1 Update interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_UP_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM1_UP_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   

	Inject_adc_init();

	SVPWM_3ShuntCurrentReadingCalibration();
 
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
} 


/*******************************************************************************
* Function Name  : SVPWM_3ShuntCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of 3 shunt resistors 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_3ShuntCurrentReadingCalibration(void)
{
  static rt_uint16_t bIndex;
  
  /* ADC1 Injected group of conversions end interrupt disabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
  
  hPhaseAOffset=0;
  hPhaseBOffset=0;
//  hPhaseCOffset=0;

  /* ADC1 Injected conversions configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC1,1);
	ADC_InjectedSequencerLengthConfig(ADC2,1);
  ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL,1,ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, PHASE_B_ADC_CHANNEL,1,ADC_SampleTime_15Cycles);
  
  /* Clear the ADC1 JEOC pending flag */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
	ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);

	ADC_SoftwareStartInjectedConv(ADC1);
	ADC_SoftwareStartInjectedConv(ADC2);
   
  /* ADC Channel used for current reading are read 
     in order to get zero currents ADC values*/ 
  for(bIndex=0; bIndex <NB_CONVERSIONS; bIndex++)
  {
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }
    hPhaseAOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)>>3);
		while(!ADC_GetFlagStatus(ADC2,ADC_FLAG_JEOC)) { }
    hPhaseBOffset += (ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1)>>3);   
        
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		

		ADC_SoftwareStartInjectedConv(ADC1);
		ADC_SoftwareStartInjectedConv(ADC2);
  }
  
  SVPWM_InjectedConvConfig();  
}

Curr_Components Get_Current_offset(void)
{
	Curr_Components Local_Currents_Offset;

	Local_Currents_Offset.qI_Component1 = hPhaseAOffset;
	Local_Currents_Offset.qI_Component2 = hPhaseBOffset;
	
	return 	Local_Currents_Offset;
}

/*******************************************************************************
* Function Name  : SVPWM_InjectedConvConfig
* Description    : This function configure ADC1 for 3 shunt current 
*                  reading and temperature and voltage feedbcak after a 
*                  calibration of the three utilized ADC Channels
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_InjectedConvConfig(void)
{
  ADC_InjectedSequencerLengthConfig(ADC1,1);
  
  ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL, 1, ADC_SampleTime_15Cycles);
	
	ADC_InjectedSequencerLengthConfig(ADC2,1);
  
  ADC_InjectedChannelConfig(ADC2, PHASE_B_ADC_CHANNEL, 1, ADC_SampleTime_15Cycles);

  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);// TIM_TRGOSource_Update

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  // Resynch to have the Update evend during Undeflow
  TIM_GenerateEvent(TIM1, TIM_EventSource_CC4);//TIM_EventSource_CC4 TIM_EventSource_Update
  // Clear Update Flag
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);//ENABLE
    
	
  /* ADC1 Injected conversions trigger is TIM1 TRGO */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4); //ADC_ExternalTrigInjecConv_T1_CC4 ADC_ExternalTrigInjecConv_T1_TRGO
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_CC4); // ADC_ExternalTrigInjecConv_T1_TRGO
  
	ADC_InjectedDiscModeCmd(ADC1,ENABLE);
	ADC_InjectedDiscModeCmd(ADC2,ENABLE);
  
  /* ADC1 Injected group of conversions end and Analog Watchdog interrupts enabling */
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntGetPhaseCurrentValues
* Description    : This function computes current values of Phase A and Phase B 
*                 in q1.15 format starting from values acquired from the A/D 
*                 Converter peripheral.
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
*******************************************************************************/
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void)
{
  Curr_Components Local_Stator_Currents;
  rt_int32_t wAux;

 // Ia = (hPhaseAOffset)-(ADC Channel 11 value)    
//	wAux = (rt_int32_t)(hPhaseAOffset)- ((ADC1->JDR1)<<1);  
	wAux = ((ADC1->JDR1)<<1) - (rt_int32_t)(hPhaseAOffset) ;	
	//wAux = ( ((ADC1->JDR1)<<1)&0xFFFFFFCF ) -  (rt_int32_t)(hPhaseAOffset) ;	
 //Saturation of Ia 
	if (wAux < S16_MIN)
	{
		Local_Stator_Currents.qI_Component1= S16_MIN;
	}  
	else  if (wAux > S16_MAX)
	{ 
		Local_Stator_Currents.qI_Component1= S16_MAX;
	}
	else
	{
		Local_Stator_Currents.qI_Component1= wAux;
	}
					 
 // Ib = (hPhaseBOffset)-(ADC Channel 12 value)
//	wAux = (rt_int32_t)(hPhaseBOffset)-((ADC2->JDR1)<<1);
	wAux = ((ADC2->JDR1)<<1) - (rt_int32_t)(hPhaseBOffset);
	//wAux = ( ((ADC2->JDR1)<<1)&0xFFFFFFCF ) -  (rt_int32_t)(hPhaseBOffset) ;	
 // Saturation of Ib
	if (wAux < S16_MIN)
	{
		Local_Stator_Currents.qI_Component2= S16_MIN;
	}  
	else  if (wAux > S16_MAX)
	{ 
		Local_Stator_Currents.qI_Component2= S16_MAX;
	}
	else
	{
		Local_Stator_Currents.qI_Component2= wAux;
	}

  return(Local_Stator_Currents); 
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
		   and configures the AD converter and TIM0 for next period 
		   current reading conversion synchronization
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
   rt_int32_t wX, wY, wZ, wUAlpha, wUBeta;
   rt_uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0;
    
   wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
   wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

   wX = wUBeta;
   wY = (wUBeta + wUAlpha)/2;
   wZ = (wUBeta - wUAlpha)/2;
   
  // Sector calculation from wX, wY, wZ
   if (wY<0)
   {
      if (wZ<0)
      {
        bSector = SECTOR_5;
      }
      else // wZ >= 0
        if (wX<=0)
        {
          bSector = SECTOR_4;
        }
        else // wX > 0
        {
          bSector = SECTOR_3;
        }
   }
   else // wY > 0
   {
     if (wZ>=0)
     {
       bSector = SECTOR_2;
     }
     else // wZ < 0
       if (wX<=0)
       {  
         bSector = SECTOR_6;
       }
       else // wX > 0
       {
         bSector = SECTOR_1;
       }
    }
   
   /* Duty cycles computation */
  PWM4Direction=PWM2_MODE;
    
  switch(bSector)
  {  
    case SECTOR_1:
        hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhB - wX/131072;
        break;
    case SECTOR_2:
        hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhA - wY/131072;
        break;
    case SECTOR_3:
        hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
				hTimePhC = hTimePhA - wY/131072;
				hTimePhB = hTimePhC + wX/131072;
        break;
    
    case SECTOR_4:
				hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhB - wX/131072;
        break;  
    case SECTOR_5:
        hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhA - wY/131072;
				break;        
    case SECTOR_6:
        hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
				hTimePhC = hTimePhA - wY/131072;
				hTimePhB = hTimePhC + wX/131072;
        break;
    default:
		break;
   }
  
  /* Load compare registers values */ 
  TIM1->CCR1 = hTimePhA;
  TIM1->CCR2 = hTimePhB;
  TIM1->CCR3 = hTimePhC;
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntAdvCurrentReading
* Description    :  It is used to enable or disable the advanced current reading.
			if disabled the current readign will be performed after update event
* Input          : cmd (ENABLE or DISABLE)
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_3ShuntAdvCurrentReading(FunctionalState cmd)
{
  if (cmd == ENABLE)
  {
    // Enable ADC trigger sync with CC4
//    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);  

	ADC1->CR2 &=0xFFC0FFFFu;
    ADC1->CR2 |=0x00100000u;
		
	ADC2->CR2 &=0xFFC0FFFFu;
    ADC2->CR2 |=0x00100000u;
    // Enable UPDATE ISR
    // Clear Update Flag
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);//ENABLE
  }
  else
  {
    // Disable UPDATE ISR
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
  // Resynch to have the Update evend during Undeflow
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);	
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
    // Sync ADC trigger with Update
//    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
//    ADC1->CR2 &=0xFFC0FFFFu;
//	ADC2->CR2 &=0xFFC0FFFFu;
 /* ADC1 Injected conversions trigger is TIM1 TRGO */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO); //ADC_ExternalTrigInjecConv_T1_CC4 ADC_ExternalTrigInjecConv_T1_TRGO
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO); // ADC_ExternalTrigInjecConv_T1_TRGO
  
	ADC_InjectedDiscModeCmd(ADC1,ENABLE);
	ADC_InjectedDiscModeCmd(ADC2,ENABLE);
  
  /* ADC1 Injected group of conversions end and Analog Watchdog interrupts enabling */
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);   
  }
}

/*******************************************************************************
* Function Name  : SVPWMUpdateEvent
* Description    :  Routine to be performed inside the update event ISR  it reenable the ext adc. triggering
		        It must be assigned to pSVPWM_UpdateEvent pointer.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWMUpdateEvent(void)
{
  // ReEnable EXT. ADC Triggering
  ADC1->CR2 |= 0x00008000;
  
  // Clear unwanted current sampling
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* Description    :  Routine to be performed inside the end of conversion ISR
		         It computes the bus voltage and temperature sensor sampling 
		        and disable the ext. adc triggering.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
rt_uint8_t SVPWMEOCEvent(void)
{
  // Store the Bus Voltage and temperature sampled values
//  h_ADCTemp = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);
//  h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
    
  if ((State == START) || (State == RUN))
  {          
    // Disable EXT. ADC Triggering
    ADC1->CR2 = ADC1->CR2 & 0xFFFF7FFF;
  }
  return ((rt_uint8_t)(1));
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
