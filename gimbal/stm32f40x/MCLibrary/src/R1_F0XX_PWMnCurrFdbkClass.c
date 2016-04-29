/**
  ******************************************************************************
  * @file    R1_F0XX_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains private implementation of R1_F0XX_PWMnCurrFdbk IRQ      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PWMnCurrFdbkClass.h"
#include "PWMnCurrFdbkPrivate.h"
#include "R1_F0XX_PWMnCurrFdbkClass.h"
#include "R1_F0XX_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* Private Defines -----------------------------------------------------------*/
#define CLASS_VARS   ((_CPWMC)this)->Vars_str

#define PWM_PERIOD (((_CPWMC) this)->pParams_str->hPWMperiod)/2u

/* Direct address of the registers used by DMA */
#define CCR1_OFFSET 0x34u
#define CCR2_OFFSET 0x38u
#define CCR3_OFFSET 0x3Cu
#define CCR4_OFFSET 0x40u
#define TIM1_CCR1_Address   TIM1_BASE + CCR1_OFFSET
#define TIM1_CCR2_Address   TIM1_BASE + CCR2_OFFSET
#define TIM1_CCR3_Address   TIM1_BASE + CCR3_OFFSET
#define TIM3_CCR4_Address   TIM3_BASE + CCR4_OFFSET
#define TIM15_CCR1_Address  TIM15_BASE + CCR1_OFFSET

#define DR_OFFSET 0x40u
#define ADC1_DR_Address     ADC1_BASE + DR_OFFSET

#define NB_CONVERSIONS 16u

#define DCLASS_PARAMS ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str

#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

#define CH1NORMAL           0x0060u
#define CH2NORMAL           0x6000u
#define CH3NORMAL           0x0060u
#define CH4NORMAL           0x7000u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u

#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u

/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((uint32_t)0x00000001u)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFEu)

#define CR2_JEXTSEL_Reset       ((uint32_t)0xFFFF8FFFu)
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00008000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set     ((u32)0x00500000)

#define ADC1_CR2_EXTTRIG_SWSTART_BB 0x42248158u

#define ADCx_IRQn     ADC1_COMP_IRQn
#define TIMx_UP_IRQn  TIM1_BRK_UP_TRG_COM_IRQn

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCR1F0XX_PWMC_t R1F0XX_PWMCpool[MAX_DRV_PWMC_NUM];
	unsigned char R1F0XX_PWMC_Allocated = 0u;
#endif

static void* R1F0XX_IRQHandler(void *this, unsigned char flag);
static void R1F0XX_Init(CPWMC this);
static void R1F0XX_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this);
static void R1F0XX_CurrentReadingCalibration(CPWMC this);
static void R1F0XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1F0XX_TurnOnLowSides(CPWMC this);
static void R1F0XX_SwitchOnPWM(CPWMC this);
static void R1F0XX_SwitchOffPWM(CPWMC this);
static void R1F0XX_1ShuntMotorVarsInit(CPWMC this);
static void R1F0XX_1ShuntMotorVarsRestart(CPWMC this);
static uint16_t R1F0XX_CalcDutyCycles(CPWMC this);
static uint16_t R1F0XX_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R1F0XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static uint16_t R1F0XX_IsOverCurrentOccurred(CPWMC this);
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin);

/**
  * @brief  Creates an object of the class R1_F0XX
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR1_F0XXParams pointer to an R1_F0XX parameters structure
  * @retval CR1F0XX_PWMC new instance of R1_F0XX object
  */
CR1F0XX_PWMC R1F0XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR1_SDParams_t pR1_SDParams)
{
	_CPWMC _oPWMnCurrFdbk;
	_DCR1F0XX_PWMC _oR1_F0XX;

	_oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oR1_F0XX = (_DCR1F0XX_PWMC)calloc(1u,sizeof(_DCR1F0XX_PWMC_t));
	#else
		if (R1F0XX_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
		{
			_oR1_F0XX = &R1F0XX_PWMCpool[R1F0XX_PWMC_Allocated++];
		}
		else
		{
			_oR1_F0XX = MC_NULL;
		}
	#endif
  
	_oR1_F0XX->pDParams_str = pR1_SDParams;
	_oPWMnCurrFdbk->DerivedClass = (void*)_oR1_F0XX;
	
	_oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R1F0XX_IRQHandler;
	Set_IRQ_Handler(pR1_SDParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R1F0XX_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R1F0XX_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R1F0XX_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R1F0XX_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &R1F0XX_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R1F0XX_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &R1F0XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &R1F0XX_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &R1F0XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &R1F0XX_CalcDutyCycles;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &R1F0XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &R1F0XX_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R1F0XX_ExecRegularConv;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R1F0XX_ADC_SetSamplingTime;
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R1F0XX_IsOverCurrentOccurred;
  
	return ((CR1F0XX_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F0XX
  * @{
  */

/** @defgroup R1_F0XX_class_private_methods R1_F0XX class private methods
* @{
*/

/**
* @brief  It initializes TIM1, ADC, GPIO, DMA1 and NVIC for single shunt current 
*         reading configuration using STM32 High Density.
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  DMA_Channel_TypeDef* DMAy_Channelx;
  ADC_InitTypeDef ADC_InitStructure;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  TIM_TypeDef* AuxTIM;
  uint16_t hAux;
  uint32_t wDMA_PeripheralBaseAddr;
  uint16_t hTIM1_CR1;
  uint16_t hAuxTIM_CR1;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  AuxTIM = pDParams_str->AuxTIM;
  
  R1F0XX_1ShuntMotorVarsInit(this);
        
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADCCLK = PCLK2 */
  RCC_ADCCLKConfig(pDParams_str->wADC_Clock_Divider);
  
  /* Enable GPIOA-GPIOF clock */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                         RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | 
                         RCC_AHBPeriph_GPIOF , ENABLE);  
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  /* Enable ADC1 clock - Used in any case for regular MC conversion */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
  /* FOC Start after DMA1_Channel2 TC - dual sampling  */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA1_Channel2_3_IRQn;
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannelPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  /* Enable DBMCU, TIM1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_DBGMCU, ENABLE);
  DBGMCU_APB2PeriphConfig(DBGMCU_TIM1_STOP, ENABLE);
  
  /* Settings related to AuxTIM used */
  if (AuxTIM == TIM3)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    DBGMCU_APB1PeriphConfig(DBGMCU_TIM3_STOP, ENABLE);
    DMAy_Channelx = DMA1_Channel3;
    wDMA_PeripheralBaseAddr = (uint32_t)TIM3_CCR4_Address;
    pDVars_str->wADC_ExtTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  } else /* TIM15 */
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM15_STOP, ENABLE);
    DMAy_Channelx = DMA1_Channel5;
    wDMA_PeripheralBaseAddr = (uint32_t)TIM15_CCR1_Address;
    pDVars_str->wADC_ExtTrigConv = ADC_ExternalTrigConv_T15_TRGO;
  }
  
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  R1F0XX_TIMxInit(TIM1, AuxTIM, this);
  
  /* DMA Event related to R1 - Active Vector insertion (TIM1 Channel 4) */
  /* DMA Channel configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM1_CCR1_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hDmaBuff);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 2u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  /* Enable DMA Channel */
  DMA_Cmd(DMA1_Channel4, ENABLE); 
  
  /* DMA Event related to AUX_TIM - dual triggering */
  /* DMA channel configuration ----------------------------------------------*/
  DMA_DeInit(DMAy_Channelx);
  DMA_InitStructure.DMA_PeripheralBaseAddr = wDMA_PeripheralBaseAddr;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hCCDmaBuffCh4);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 3u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMAy_Channelx, &DMA_InitStructure);
  /* Enable DMA Channel */
  DMA_Cmd(DMAy_Channelx, ENABLE);
  
  /* DMA Event related to ADC conversion*/
  /* DMA channel configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hCurConv);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  /* Remap ADC DMA channel */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_ADC1, ENABLE); /* ADC on DMA1_CH2 */
  /* DMA1 channel 2 will be enabled after the CurrentReadingCalibration */
  
  /* Enable the TIM1 Update interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIMx_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
  
  if (pDParams_str->bRepetitionCounter > 1u)
  {
    /* Only if REP RATE > 1 - Active Vector insertion (TIM1 Channel 4)*/
    /* enable the DMA1_CH4 TC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA1_Channel4_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = DMAx_TC_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable DMA1 CH4 TC IRQ */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    
    pDVars_str->bDMATot = (pDParams_str->bRepetitionCounter+1u)/2u;
  }
  else
  {
    /* REP RATE = 1 */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
    pDVars_str->bDMATot = 0u;
  }
        
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(pDParams_str->hIPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIPort, pDParams_str->hIPin);
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/
  /* Note: in F0xx TIM1 is mapped on AF2 */
  GPIO_PinAFConfig(pDParams_str->hCh1Port, F0XX_GPIOPin2Source(pDParams_str->hCh1Pin), GPIO_AF_2);
  GPIO_PinAFConfig(pDParams_str->hCh2Port, F0XX_GPIOPin2Source(pDParams_str->hCh2Pin), GPIO_AF_2);
  GPIO_PinAFConfig(pDParams_str->hCh3Port, F0XX_GPIOPin2Source(pDParams_str->hCh3Pin), GPIO_AF_2);
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1Pin;
  GPIO_Init(pDParams_str->hCh1Port, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2Pin;
  GPIO_Init(pDParams_str->hCh2Port, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3Pin;
  GPIO_Init(pDParams_str->hCh3Port, &GPIO_InitStructure);

  GPIO_PinLockConfig(pDParams_str->hCh1Port, pDParams_str->hCh1Pin);
  GPIO_PinLockConfig(pDParams_str->hCh2Port, pDParams_str->hCh2Pin);
  GPIO_PinLockConfig(pDParams_str->hCh3Port, pDParams_str->hCh3Pin);
  
  /****** Configure TIMx Channel 1N, 2N and 3N Outputs, if enabled ******/    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  {
    /* Note: in F0xx TIM1 is mapped on AF2 */
    GPIO_PinAFConfig(pDParams_str->hCh1NPort, F0XX_GPIOPin2Source(pDParams_str->hCh1NPin), GPIO_AF_2);
    GPIO_PinAFConfig(pDParams_str->hCh2NPort, F0XX_GPIOPin2Source(pDParams_str->hCh2NPin), GPIO_AF_2);
    GPIO_PinAFConfig(pDParams_str->hCh3NPort, F0XX_GPIOPin2Source(pDParams_str->hCh3NPin), GPIO_AF_2);
    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2NPin;  
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3NPin;  
    GPIO_Init(pDParams_str->hCh3NPort, &GPIO_InitStructure);
    
    /* Lock low side configuration registers */
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin);
    GPIO_PinLockConfig(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin);    
  }  else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Only "active high" polarity is supported */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2NPin;  
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3NPin;  
    GPIO_Init(pDParams_str->hCh3NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin);
  }
  else
  {
  }  
      
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    /****** Configure TIMx BKIN input, if enabled ******/
    /* Note: in F0xx TIM1 is mapped on AF2 */
    GPIO_PinAFConfig(pDParams_str->hBKINPort, F0XX_GPIOPin2Source(pDParams_str->hBKINPin), GPIO_AF_2);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
    /* Clear TIMx break flag. */
    TIM_ClearFlag(TIM1,TIM_FLAG_Break);
  }
  
  ADC_StructInit(&ADC_InitStructure);
  /* ADC registers configuration -----------------------------------*/
  /* ADC registers reset */  
  ADC_DeInit(ADC1);
  
  /* Start calibration of ADC1 */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADC */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Wait ADC Ready */
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)==RESET)
  {}  
  
  /* ADC Init */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* Enable ADC1 EOC DMA */
  ADC_DMACmd(ADC1,ENABLE);
  
  R1F0XX_1ShuntMotorVarsRestart(this);
  
  /* Set AUX TIM channel first trigger (dummy) - DMA enabling */
  hAux = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  if (AuxTIM == TIM3)
  {
    TIM3->CCR4 = hAux;
    TIM_DMACmd(TIM3, TIM_DMA_CC4, ENABLE);
  }
  else /* TIM15 */
  {
    TIM15->CCR1 = hAux;
    TIM_DMACmd(TIM15, TIM_DMA_CC1, ENABLE);
  }
  
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
  
  TIM_Cmd(TIM1,ENABLE);
  TIM_Cmd(AuxTIM,ENABLE);
  
  hTIM1_CR1 = TIM1->CR1;
  hTIM1_CR1 |= TIM_CR1_CEN;
  hAuxTIM_CR1 = AuxTIM->CR1;
  hAuxTIM_CR1 |= TIM_CR1_CEN;
  
  AuxTIM->CNT += 3u;
  
  __disable_irq();
  TIM1->CR1 = hTIM1_CR1;
  AuxTIM->CR1 = hAuxTIM_CR1;
  __enable_irq();
}

/**
* @brief  It initializes TIMx and TIMx_2 peripheral for PWM generation, 
          active vector insertion and adc triggering.
* @param  TIMx Timer to be initialized
* @param  TIMx_2 Auxiliary timer to be initialized used for adc triggering
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDParams_t pDParams_str;
  uint16_t hAux;
  
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_DeInit(TIMx_2);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
  hAux = PWM_PERIOD;
  TIMx_TimeBaseStructure.TIM_Period = hAux;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams_str->bRepetitionCounter;
  TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  hAux = (PWM_PERIOD * 2u) - 1u;
  TIMx_TimeBaseStructure.TIM_Period = hAux;
  TIM_TimeBaseInit(TIMx_2, &TIMx_TimeBaseStructure);
    
  /* Channel 1, 2,3 Configuration in PWM mode */
  TIM_OCStructInit(&TIMx_OCInitStructure);  
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse = 0x0u; /* dummy value */
  
  /* Channel 1 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh1Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh1IdleState;    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh1NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh1NIdleState;     
  }    
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
  TIM_OC1Init(TIMx, &TIMx_OCInitStructure); 
    
  /* Channel 2 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh2Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh2IdleState;   
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh2NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh2NIdleState;         
  }
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }
  TIM_OC2Init(TIMx, &TIMx_OCInitStructure); 
    
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh3IdleState; 
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh3NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh3NIdleState;         
  }
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }
  TIM_OC3Init(TIMx, &TIMx_OCInitStructure);   
  
  /* Channel 4 Configuration in PWM mode */
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; /* TIM_OutputState_Enable */
  hAux = PWM_PERIOD-pDParams_str->hHTMin;
  TIMx_OCInitStructure.TIM_Pulse = hAux;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC4Init(TIMx, &TIMx_OCInitStructure);
    
  /* Dead Time */
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pDParams_str->hDeadTime)/2u;
  /* BKIN, if enabled */
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    TIMx_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pDParams_str->hBKINPolarity;
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_ClearITPendingBit(TIMx, TIM_IT_Break);
  }  
  TIM_BDTRConfig(TIMx, &TIMx_BDTRInitStructure);
  
  /* Disable update interrupt */
  TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
  
  TIM_SelectOutputTrigger(TIMx, TIM_TRGOSource_Update);
  
  /* TIMx_2 channel Init */
  hAux = PWM_PERIOD >> 2u - pDParams_str->hTMin - pDParams_str->hTbefore;
  TIMx_OCInitStructure.TIM_Pulse = hAux;
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; /* Enable here for sampling point debug */
  if (TIMx_2 == TIM3)
  {
    TIM_OC4Init(TIMx_2, &TIMx_OCInitStructure);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC4Ref);
  }
  else /* TIM15 */
  {
    TIM_OC1Init(TIMx_2, &TIMx_OCInitStructure);
    TIM_SelectOutputTrigger(TIM15, TIM_TRGOSource_OC1Ref);
  }
    
  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIMx,TIM_EventSource_Update);
  TIM_GenerateEvent(TIMx_2,TIM_EventSource_Update);
}

/**
* @brief  It stores into 'this' object variables the voltage present on the  
*         current feedback analog channel when no current is flowin into the
*         motor
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_CurrentReadingCalibration(CPWMC this)
{
  uint8_t bIndex = 0u;
  uint32_t wPhaseOffset = 0u;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  uint32_t wAux;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Set the CALIB flags to indicate the ADC calibartion phase*/
  pDVars_str->hFlags |= CALIB;
  
  /* ADC Channel and sampling time config for current reading */
  wAux = 1u;
  ADC1->CHSELR = wAux << pDParams_str->hIChannel;
  ADC1->SMPR = (uint32_t)(pDParams_str->b_ISamplingTime) & 0x00000007u;
  
  /* Disable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, DISABLE);
  
  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/   
  while (bIndex< NB_CONVERSIONS)
  {     
    /* Software start of conversion */
    ADC_StartOfConversion(ADC1);
    
    /* Wait until end of regular conversion */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET)
    {}    
    
    wPhaseOffset += ADC_GetConversionValue(ADC1);
    bIndex++;
  }
  
  pDVars_str->hPhaseOffset = (uint16_t)(wPhaseOffset/NB_CONVERSIONS);
  
  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pDVars_str->hFlags &= (~CALIB);
  
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

/**
* @brief  First initialization of class members
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_1ShuntMotorVarsInit(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Init motor vars */
  pDVars_str->hPhaseOffset=0u;
  pDVars_str->bInverted_pwm=INVERT_NONE;
  pDVars_str->bInverted_pwm_new=INVERT_NONE;
  pDVars_str->hFlags &= (~STBD3);
  pDVars_str->hFlags &= (~DSTEN);
  
  /* After reset value of DMA buffers */
  pDVars_str->hDmaBuff[0] = PWM_PERIOD + 1u;
  pDVars_str->hDmaBuff[1] = PWM_PERIOD >> 1;
    
  /* After reset value of dvDutyValues */
  CLASS_VARS.hCntPhA = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhB = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhC = PWM_PERIOD >> 1;
  
  /* Default value of DutyValues */
  pDVars_str->hCntSmp1 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  pDVars_str->hCntSmp2 = (PWM_PERIOD >> 1) + pDParams_str->hTafter;
  
  /* Default value of sampling point */
  pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
  pDVars_str->hCCDmaBuffCh4[1] = (PWM_PERIOD * 2u) - 1u;         /* Update */
  pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  
  /* Init of "regular" conversion registers */ 
  pDVars_str->bRegConvRequested = 0u;
  pDVars_str->bRegConvIndex = 0u;
  
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  Initialization of class members after each motor start
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_1ShuntMotorVarsRestart(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Default value of DutyValues */
  pDVars_str->hCntSmp1 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  pDVars_str->hCntSmp2 = (PWM_PERIOD >> 1) + pDParams_str->hTafter;
  
  /* Default value of sampling point */
  pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
  pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  
  /* After start value of DMA buffers */
  pDVars_str->hDmaBuff[0] = PWM_PERIOD + 1u;
  pDVars_str->hDmaBuff[1]= PWM_PERIOD >> 1;
  
  /* After start value of dvDutyValues */
  CLASS_VARS.hCntPhA = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhB = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhC = PWM_PERIOD >> 1;
  
  /* Set the default previous value of Phase A,B,C current */
  pDVars_str->hCurrAOld=0;
  pDVars_str->hCurrBOld=0;
  pDVars_str->hCurrCOld=0;
    
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this related object of class CPWMC
* @retval Curr_Components Ia and Ib current in Curr_Components format
*/
static void R1F0XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{  
  int32_t wAux;
  uint32_t wuAux;
  int16_t hCurrA = 0, hCurrB = 0, hCurrC = 0;
  uint8_t bCurrASamp = 0u, bCurrBSamp = 0u, bCurrCSamp = 0u;
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
    
  /* Disabling the External triggering for ADCx*/
  ADC1->CFGR1 &= ~ADC_ExternalTrigConvEdge_RisingFalling;
  
  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC);
  
  /* First sampling point */
  wAux = (int32_t)(pDVars_str->hCurConv[0]);
  wAux -= (int32_t)(pDVars_str->hPhaseOffset);
  
  /* Check saturation */
  if (wAux > S16_MIN)
  {
    if (wAux < S16_MAX)
    {
    }
    else
    {
      wAux = S16_MAX;
    }
  }
  else
  {
    wAux = S16_MIN;
  }  
  
  switch (pDVars_str->sampCur1)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux;
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux;
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux;
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_OLDA:
    hCurrA = pDVars_str->hCurrAOld;
    bCurrASamp = 1u;
    break;
  case SAMP_OLDB:
    hCurrB = pDVars_str->hCurrBOld;
    bCurrBSamp = 1u;
    break;
  default:
    break;
  }
  
  /* Second sampling point */
  wAux = (int32_t)(pDVars_str->hCurConv[1]);
  wAux -= (int32_t)(pDVars_str->hPhaseOffset);
  
  /* Check saturation */
  if (wAux > S16_MIN)
  {
    if (wAux < S16_MAX)
    {
    }
    else
    {
      wAux = S16_MAX;
    }
  }
  else
  {
    wAux = S16_MIN;
  }
  
  switch (pDVars_str->sampCur2)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux; 
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux; 
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux; 
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  default:
    break;
  }
    
  /* Computation of the third value */
  if (bCurrASamp == 0u)
  {
    wAux = -((int32_t)(hCurrB)) -((int32_t)(hCurrC));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrA = (int16_t)wAux; 
  }
  if (bCurrBSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrC));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrB = (int16_t)wAux;
  }
  if (bCurrCSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrB));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrC = (int16_t)wAux;
  }
  
  /* hCurrA, hCurrB, hCurrC values are the sampled values */
    
  pDVars_str->hCurrAOld = hCurrA;
  pDVars_str->hCurrBOld = hCurrB;
  pDVars_str->hCurrCOld = hCurrC;
  
  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
  
  if (pDVars_str->bRegConvRequested != 0u)
  {
    /* Exec regular conversion @ bRegConvIndex */
    uint8_t bRegConvCh = pDVars_str->bRegConvCh[pDVars_str->bRegConvIndex];
    
    /* Set Sampling time and channel */
    wuAux = 1u;
    ADC1->CHSELR = wuAux <<  bRegConvCh;
    ADC1->SMPR = (uint32_t)(pDVars_str->bRegSmpTime[bRegConvCh]) & 0x00000007u;
    
    /* Enable ADC1 EOC DMA */
    ADC_DMACmd(ADC1,DISABLE);
    
    /* Start ADC */
    ADC_StartOfConversion(ADC1);
    
    /* Flags the regular conversion ongoing */
    pDVars_str->hFlags |= REGCONVONGOING;
  }
}

/**
* @brief  It turns on low sides switches. This function is intended to be 
*         used for charging boot capacitors of driving section. It has to be 
*         called each motor start-up when using high voltage drivers
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_TurnOnLowSides(CPWMC this)
{
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;
  
  TIM_ClearFlag(TIM1,TIM_FLAG_Update);
  while (TIM_GetFlagStatus(TIM1,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  return; 
}

/**
* @brief  This function enables the update event and the single shunt distortion
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_SwitchOnPWM(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  
  /* Enable UPDATE ISR */
  /* Clear Update Flag */
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  
  /* Enabling distortion for single shunt */
  pDVars_str->hFlags |= DSTEN;
  return; 
}

/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit, disables the single shunt distortion and reset the TIM status
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F0XX_SwitchOffPWM(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  uint16_t hAux;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_RESET);
  }
  
  /* Disable UPDATE ISR */
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    
  /* Disabling distortion for single */
  pDVars_str->hFlags &= (~DSTEN);

  while (TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)==RESET)
  {}
  /* Disabling all DMA previous setting */
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);  
  
  /* Set all duty to 50% */
  hAux = PWM_PERIOD >> 1;
  TIM1->CCR1 = hAux;
  TIM1->CCR2 = hAux;
  TIM1->CCR3 = hAux;    
    
  return; 
}

/**
* @brief  Implementation of the single shunt algorithm to setup the 
*         TIM1 register and DMA buffers values for the next PWM period.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs 
          before the end of FOC algorithm else returns MC_NO_ERROR
*/
static uint16_t R1F0XX_CalcDutyCycles(CPWMC this)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;
  uint32_t wAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
    
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  bSector = (uint8_t)(((_CPWMC)this)->Vars_str.hSector);
  
  if ((pDVars_str->hFlags & DSTEN) != 0u)
  { 
    switch (bSector)
    {
    case SECTOR_1:
      hDutyV_2 = CLASS_VARS.hCntPhA;
      hDutyV_1 = CLASS_VARS.hCntPhB;
      hDutyV_0 = CLASS_VARS.hCntPhC;
      break;
    case SECTOR_2:
      hDutyV_2 = CLASS_VARS.hCntPhB;
      hDutyV_1 = CLASS_VARS.hCntPhA;
      hDutyV_0 = CLASS_VARS.hCntPhC;
      break;
    case SECTOR_3:
      hDutyV_2 = CLASS_VARS.hCntPhB;
      hDutyV_1 = CLASS_VARS.hCntPhC;
      hDutyV_0 = CLASS_VARS.hCntPhA;
      break;
    case SECTOR_4:
      hDutyV_2 = CLASS_VARS.hCntPhC;
      hDutyV_1 = CLASS_VARS.hCntPhB;
      hDutyV_0 = CLASS_VARS.hCntPhA;
      break;
    case SECTOR_5:
      hDutyV_2 = CLASS_VARS.hCntPhC;
      hDutyV_1 = CLASS_VARS.hCntPhA;
      hDutyV_0 = CLASS_VARS.hCntPhB;
      break;
    case SECTOR_6:
      hDutyV_2 = CLASS_VARS.hCntPhA;
      hDutyV_1 = CLASS_VARS.hCntPhC;
      hDutyV_0 = CLASS_VARS.hCntPhB;
      break;
    default:
      break;
    }
    
    /* Compute delta duty */
    hDeltaDuty_0 = (int16_t)(hDutyV_1) - (int16_t)(hDutyV_0);
    hDeltaDuty_1 = (int16_t)(hDutyV_2) - (int16_t)(hDutyV_1);
    
    /* Check region */
    if ((uint16_t)hDeltaDuty_0<=pDParams_str->hTMin)
    {
      if ((uint16_t)hDeltaDuty_1<=pDParams_str->hTMin)
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    } 
    else 
    {
      if ((uint16_t)hDeltaDuty_1>pDParams_str->hTMin)
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }
            
    if (bStatorFluxPos == REGULAR)
    {
      pDVars_str->bInverted_pwm_new = INVERT_NONE;
    }
    else if (bStatorFluxPos == BOUNDARY_1) /* Adjust the lower */
    {
      switch (bSector)
      {
      case SECTOR_5:
      case SECTOR_6:
        if (CLASS_VARS.hCntPhA - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_A;
          CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhA < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhA;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_2:
      case SECTOR_1:
        if (CLASS_VARS.hCntPhB - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_B;
          CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhB < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhB;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_4:
      case SECTOR_3:
        if (CLASS_VARS.hCntPhC - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_C;
          CLASS_VARS.hCntPhC -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhC < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhC;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      default:
        break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_2) /* Adjust the middler */
    {
      switch (bSector)
      {
      case SECTOR_4:
      case SECTOR_5: /* Invert B */
        pDVars_str->bInverted_pwm_new = INVERT_B;
        CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhB > 0xEFFFu)
        {
          CLASS_VARS.hCntPhB = 0u;
        }
        break;
      case SECTOR_2:
      case SECTOR_3: /* Invert A */
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhA > 0xEFFFu)
        {
          CLASS_VARS.hCntPhA = 0u;
        }
        break;
      case SECTOR_6:
      case SECTOR_1: /* Invert C */
        pDVars_str->bInverted_pwm_new = INVERT_C;
        CLASS_VARS.hCntPhC -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhC > 0xEFFFu)
        {
          CLASS_VARS.hCntPhC = 0u;
        }
        break;
      default:
        break;
      }
    }
    else
    {
      if ((pDVars_str->hFlags & STBD3) == 0u)
      {
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
        pDVars_str->hFlags |= STBD3;
      } 
      else
      {
        pDVars_str->bInverted_pwm_new = INVERT_B;
        CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
        pDVars_str->hFlags &= (~STBD3);
      }
    }
        
    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_0 + hDutyV_1 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      }
      /* Second point */
      if ((hDutyV_2 - hDutyV_1 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp2 = hDutyV_1 + hDutyV_2 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp2 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp2 = hDutyV_2 - pDParams_str->hTbefore;
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_0 + hDutyV_1 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      }
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
      if ((hDutyV_2 - hDutyV_1 - pDParams_str->hDeadTime)>= pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_1 + hDutyV_2 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_2 - pDParams_str->hTbefore;
      }
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_0-pDParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
  }
  else
  {
    pDVars_str->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }
    
  /* Update Timer Ch 1,2,3 (These value are required before update event) */
    
  pDVars_str->hFlags |= EOFOC;
  /* Check if DMA transition has been completed */
  if (pDVars_str->bDMACur == 0u)
  {    
    /* Preload Enable */
    TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
    
    TIM1->CCR1 = CLASS_VARS.hCntPhA;
    TIM1->CCR2 = CLASS_VARS.hCntPhB;
    TIM1->CCR3 = CLASS_VARS.hCntPhC;

    /* Update ADC Trigger DMA buffer */    
    pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
    pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  }
  
  if ((pDVars_str->hFlags & REGCONVONGOING)==0u)
  {
  }
  else
  {
    pDVars_str->hRegConvValue[pDVars_str->bRegConvIndex] = (uint16_t)(ADC1->DR);
    
    /* ADC Channel and sampling time config for current reading */
    wAux = 1u;
    ADC1->CHSELR = wAux <<  pDParams_str->hIChannel;
    ADC1->SMPR = (uint32_t)(pDParams_str->b_ISamplingTime) & 0x00000007u;
    
    /* Enable ADC1 EOC DMA */
    ADC_DMACmd(ADC1,ENABLE);
    
    /* Clear regular conversion ongoing flag */
    pDVars_str->hFlags &= (uint16_t)~REGCONVONGOING;
    
    /* Prepare next conversion */
    pDVars_str->bRegConvIndex++;
    
    if (pDVars_str->bRegConvIndex >= pDVars_str->bRegConvRequested)
    {
      pDVars_str->bRegConvIndex = 0u;
    }
  }
      
  /* Limit for update event */
  
  /* Check the status of bSOFOC flags if is set the next update event has been 
  occurred so an error will be reported*/
  if ((pDVars_str->hFlags & SOFOC) != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  /* The following instruction can be executed after Update handler 
     before the get phase current (Second EOC) */
      
  /* Set the current sampled */
   if (bStatorFluxPos == REGULAR) /* Regual zone */
  {
    pDVars_str->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
  {
    pDVars_str->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
  {
    pDVars_str->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_3)  
  {
    if (pDVars_str->bInverted_pwm_new == INVERT_A)
    {
      pDVars_str->sampCur1 = SAMP_OLDB;
      pDVars_str->sampCur2 = SAMP_IA;
    }
    if (pDVars_str->bInverted_pwm_new == INVERT_B)
    {
      pDVars_str->sampCur1 = SAMP_OLDA;
      pDVars_str->sampCur2 = SAMP_IB;
    }
  }
    
  /* Limit for the Get Phase current (Second EOC Handler) */
      
  return (hAux);
}

/**
  * @brief  R1_F0XX implement MC IRQ function TIMER Update and DMA TC
  * @param  this related object
  * @param  flag used to indicate which IRQ has been occurred
  *			0 Means TIM1 Update IRQ occurred
  *			1 Not used
  *			2 Means DAC TC IRQ occurred
  * @retval void* It returns always MC_NULL
  */
static void* R1F0XX_IRQHandler(void* this, unsigned char flag)
{   
  uint8_t bInverted_pwm_new;
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  switch (flag) /* Case 1 is not used */
  {
  case 0: /* TIM1 Update IRQ */
    { 
      uint32_t wAux;
      /* Critical point start */
      
      /* Enabling the External triggering for ADCx*/
      /* F0xx TIM15_CC1 -> TIM15_TRG0 */
      wAux = ADC1->CFGR1;
      wAux &= (~(ADC_CFGR1_EXTEN | ADC_CFGR1_EXTSEL));
      wAux |= (ADC_ExternalTrigConvEdge_Rising | pDVars_str->wADC_ExtTrigConv);
      ADC1->CFGR1 = wAux;
      /* Enable ADC triggering */
      ADC1->CR |= (uint32_t)ADC_CR_ADSTART;
      
      /* Critical point stop */
      
      /* TMP var to speedup the execution */
      bInverted_pwm_new = pDVars_str->bInverted_pwm_new;
      
      if (bInverted_pwm_new != pDVars_str->bInverted_pwm)  
      {
        /* Set the DMA destination */
        switch (bInverted_pwm_new)
        {
        case INVERT_A:
          DMA1_Channel4->CPAR = TIM1_CCR1_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_B:
          DMA1_Channel4->CPAR = TIM1_CCR2_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_C:
          DMA1_Channel4->CPAR = TIM1_CCR3_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        default:
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);*/
          TIM1->DIER &= (uint16_t)~TIM_DMA_CC4;
          break;
        }  
      }
      
      /* Clear of End of FOC Flags */
      pDVars_str->hFlags &= (~EOFOC);
      
      /* Preload Disable */
      TIM1->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
      TIM1->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;
      
      switch (bInverted_pwm_new)
      {
      case INVERT_A:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhA;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      case INVERT_B:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhB;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      case INVERT_C:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhC;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      default:
        pDVars_str->bDMACur = 0u;
        break;
      }
      
      pDVars_str->bInverted_pwm = bInverted_pwm_new;      
    
      /* Set the bSOFOC flags to indicate the execution of Update IRQ*/
      pDVars_str->hFlags |= SOFOC;    
    }
    break;
  case 2: /* DMA TC IRQ */
    {
      pDVars_str->bDMACur--;
      if (pDVars_str->bDMACur == 0u)
      {
        if ((pDVars_str->hFlags & EOFOC) != 0u)
        {
          /* Preload Enable */
          TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
          TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
          
          /* Compare register update */
          TIM1->CCR1 = CLASS_VARS.hCntPhA;
          TIM1->CCR2 = CLASS_VARS.hCntPhB;
          TIM1->CCR3 = CLASS_VARS.hCntPhC;
          
          /* Update ADC Trigger DMA buffer */    
          pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
          pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */        
        }
      }
    }
    break;
  default:
    break;
  }
  return MC_NULL;
}

/**
* @brief  Execute a regular conversion. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval uint16_t It returns converted value or oxFFFF for conversion error */
static uint16_t R1F0XX_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  pDVars_t pDVars_str = &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  uint16_t hRetVal = 0xFFFFu;
  uint8_t i;
  bool bRegChFound = FALSE;
  uint32_t wAux;
  
  if (bChannel < 18u)
  {
    /* Check if the channel has been already requested */
    for (i = 0u; i < pDVars_str->bRegConvRequested; i++)
    {
      if (pDVars_str->bRegConvCh[i] == bChannel)
      {
        hRetVal = pDVars_str->hRegConvValue[i];
        bRegChFound = TRUE;
        break;
      }
    }
    if (bRegChFound == FALSE)
    {
      if (pDVars_str->bRegConvRequested < MAX_REG_CONVERSIONS)
      {
        /* Add new channel to the list */
        pDVars_str->bRegConvCh[pDVars_str->bRegConvRequested] = bChannel;
        i = pDVars_str->bRegConvRequested;
        pDVars_str->bRegConvRequested++;
      }
    }
    if ((pDVars_str->hFlags & CALIB) == 0u)
    {
      if ((TIM1->DIER & TIM_IT_Update)!= TIM_IT_Update)
      {
        /* The current reading is disabled (TIM_IT_Update = 0) */
        /* Start the "regular" conversion immediately */
        
        /* Set Sampling time and channel */
        wAux = 1u;
        ADC1->CHSELR = wAux <<  bChannel;
        ADC1->SMPR = (uint32_t)(pDVars_str->bRegSmpTime[bChannel]) & 0x00000007u;
        
        /* Disable ADC1 EOC DMA */
        ADC_DMACmd(ADC1,DISABLE);
        
        /* Disabling the External triggering for ADCx*/
        ADC1->CFGR1 &= ~ADC_ExternalTrigConvEdge_RisingFalling;
        
        /* Clear EOC */
        ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
        
        /* Start ADC */
        ADC_StartOfConversion(ADC1);
        
        /* Wait EOC */
        while (ADC_GetFlagStatus(ADC1,ADC_IT_EOC) == RESET)
        {
        }
        
        /* Read the "Regular" conversion (Not related to current sampling) */
        hRetVal = ADC_GetConversionValue(ADC1);
        pDVars_str->hRegConvValue[i] = hRetVal;
        
        /* Enable ADC1 EOC DMA */
        ADC_DMACmd(ADC1,ENABLE);
      }
    }
  }
  
  return hRetVal;
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
static void R1F0XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{
  pDVars_t pDVars_str = &(((_DCR1F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  if (ADConv_struct.Channel < 18u)
  {
    if (ADConv_struct.SamplTime < 8u)
    {
      pDVars_str->bRegSmpTime[ADConv_struct.Channel] = ADConv_struct.SamplTime;
    }
  }
}

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
static uint16_t R1F0XX_IsOverCurrentOccurred(CPWMC this)
{
  uint16_t retVal = MC_NO_FAULTS;
  if ((TIM1->SR & TIM_FLAG_Break) != 0u)
  {
    retVal = MC_BREAK_IN;
    TIM1->SR = (uint16_t)~TIM_FLAG_Break;
  }
  return retVal;
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin)
{
  uint16_t hRetVal;
  switch (GPIO_Pin)
  {
  case GPIO_Pin_0:
    {
      hRetVal = GPIO_PinSource0;
      break;
    }
  case GPIO_Pin_1:
    {
      hRetVal = GPIO_PinSource1;
      break;
    }
  case GPIO_Pin_2:
    {
      hRetVal = GPIO_PinSource2;
      break;
    }
  case GPIO_Pin_3:
    {
      hRetVal = GPIO_PinSource3;
      break;
    }
  case GPIO_Pin_4:
    {
      hRetVal = GPIO_PinSource4;
      break;
    }
  case GPIO_Pin_5:
    {
      hRetVal = GPIO_PinSource5;
      break;
    }
  case GPIO_Pin_6:
    {
      hRetVal = GPIO_PinSource6;
      break;
    }
  case GPIO_Pin_7:
    {
      hRetVal = GPIO_PinSource7;
      break;
    }
  case GPIO_Pin_8:
    {
      hRetVal = GPIO_PinSource8;
      break;
    }
  case GPIO_Pin_9:
    {
      hRetVal = GPIO_PinSource9;
      break;
    }
  case GPIO_Pin_10:
    {
      hRetVal = GPIO_PinSource10;
      break;
    }
  case GPIO_Pin_11:
    {
      hRetVal = GPIO_PinSource11;
      break;
    }
  case GPIO_Pin_12:
    {
      hRetVal = GPIO_PinSource12;
      break;
    }
  case GPIO_Pin_13:
    {
      hRetVal = GPIO_PinSource13;
      break;
    }
  case GPIO_Pin_14:
    {
      hRetVal = GPIO_PinSource14;
      break;
    }
  case GPIO_Pin_15:
    {
      hRetVal = GPIO_PinSource15;
      break;
    }
  default:
    {
      hRetVal = 0u;
      break;
    }
  }
  return hRetVal;
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
