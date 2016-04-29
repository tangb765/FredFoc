/**
  ******************************************************************************
  * @file    ICS_F30X_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains private definition of ICS_F30X class      
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
#ifndef __ICS_F30X_PWMNCURRFDBKPRIVATE_H
#define __ICS_F30X_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_ICS_F30X
  * @{
  */

/** @defgroup ICS_F30X_Class_Private_Defines ICS_F30X private defines
* @{
*/
/** 
* @brief  Flags definition
*/

/**
* @}
*/
/** @defgroup ICS_F30X_private_types ICS_F30X private types
* @{
*/

/** 
  * @brief  ICS_F30X class members definition 
  */
typedef struct
{
  void *    pDrive;     /* Pointer to drive object related to PWMnCurr object. 
                          It is returned by the MC TIMx update IRQ handler */
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef ICS_DDParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private ICS_F30X class definition 
  */
typedef struct
{
   DVars_t DVars_str;		/*!< Derived class members container */
   pDParams_t pDParams_str;	/*!< Derived class parameters container */      
}_DCIF30X_PWMC_t, *_DCIF30X_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ICS_F30X_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
