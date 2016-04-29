/**
  ******************************************************************************
  * @file    ICS_F30X_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains interface of ICS_F30X class      
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_F30X_PWMNCURRFDBKCLASS_H
#define __ICS_F30X_PWMNCURRFDBKCLASS_H

#include "ICS_DD_PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_ICS_F30X
  * @{
  */

/** @defgroup ICS_F30X_class_exported_types ICS_F30X class exported types
* @{
*/

/** 
  * @brief  Public ICS_F30X class definition
  */
typedef struct CIF30X_PWMC_t *CIF30X_PWMC;

/**
  * @}
  */

/** @defgroup ICS_F30X_class_exported_methods ICS_F30X class exported 
  *           methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class ICS_F30X
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pICS_DDParams pointer to an ICS_DD parameters structure
  * @retval CIF30X_PWMC new instance of ICS_F30X object
  */
CIF30X_PWMC IF3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                              pICS_DDParams_t pICS_DDParams);

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
void IF3XX_StartTimers(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ICS_F30X_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
