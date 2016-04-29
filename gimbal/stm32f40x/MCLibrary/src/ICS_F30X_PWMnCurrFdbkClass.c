/**
  ******************************************************************************
  * @file    ICS_F30X_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file will contains implementation of current sensor class to be
  *          instantiated when an insulated current sensing topology is 
  *          used.
  *          It is specifically designed for STM32F30x microcontrollers.
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
#include "ICS_F30X_PWMnCurrFdbkClass.h"
#include "ICS_F30X_PWMnCurrFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_PARAMS ((_DCIF30X_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCIF30X_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCIF30X_PWMC_t IF30X_PWMCpool[MAX_DRV_PWMC_NUM];
unsigned char IF30X_PWMC_Allocated = 0u;
#endif

/**
* @brief  Creates an object of the class ICS_F30X
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param  pICS_DDParams pointer to an ICS_DD parameters structure
* @retval CIF30X_PWMC new instance of ICS_F30X object
*/
CIF30X_PWMC IF3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pICS_DDParams_t pICS_DDParams)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCIF30X_PWMC _oICS_F30X;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oICS_F30X = (_DCIF30X_PWMC)calloc(1u,sizeof(_DCIF30X_PWMC_t));
#else
  if (IF30X_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oICS_F30X = &IF30X_PWMCpool[IF30X_PWMC_Allocated++];
  }
  else
  {
    _oICS_F30X = MC_NULL;
  }
#endif
  
  _oICS_F30X->pDParams_str = pICS_DDParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oICS_F30X;
  
  return ((CIF30X_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk_ICS_F30X
* @{
*/

/** @defgroup ICS_F30X_class_private_methods ICS_F30X class private methods
* @{
*/

/**
* @brief  It perform the start of all the timers required by the control. 
          It utilizes TIM2 as temporary timer to achieve synchronization between 
          PWM signals.
          When this function is called, TIM1 and/or TIM8 must be in frozen state
          with CNT, ARR, REP RATE and trigger correctly set (these setting are 
          usually performed in the Init method accordingly with the configuration)
* @param  none
* @retval none
*/
 void IF3XX_StartTimers(void)
{
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
