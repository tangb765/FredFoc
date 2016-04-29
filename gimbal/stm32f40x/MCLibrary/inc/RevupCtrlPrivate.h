/**
  ******************************************************************************
  * @file    RevupCtrlPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains private definition of RevupCtrl class      
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
#ifndef __REVUPCTRLPRIVATE_H
#define __REVUPCTRLPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup RevupCtrl
  * @{
  */

/** @defgroup RevupCtrl_class_private_types RevupCtrl class private types
* @{
*/

#define RUC_ALLOWS_TUNING

#ifdef RUC_ALLOWS_TUNING

  #define MAX_PHASE_NUMBER 8u

#endif

#ifdef RUC_ALLOWS_TUNING
  typedef struct
  {
    uint16_t hDurationms;       /*!< Duration of the rev up phase expressed in
    milliseconds.*/
    int16_t hFinalMecSpeed01Hz; /*!< Mechanical speed expressed in 0.1Hz assumed
    by VSS at the end of the rev up phase.*/
    int16_t hFinalTorque;       /*!< Motor torque reference imposed by STC at the
    end of rev up phase. This value represents
    actually the Iq current expressed in digit.*/
    void* pNext;                /*!< Pointer of the next element of phase params.
    It can be MC_NULL for the last element.*/
  } RUCPhasesData_t, *pRUCPhasesData_t;  
#endif

/** 
  * @brief  RevupCtrl class members definition
  */
typedef struct
{
  CSTC oSTC;                     /*!< Speed and torque controller object used by
                                      RUC.*/
  CVSS_SPD oVSS;                 /*!< Virtual speed sensor object used by RUC.*/
  uint16_t hPhaseRemainingTicks; /*!< Number of clock events remaining to
                                      complete the phase.*/
  pRUCPhasesParams_t pPhaseParams;/*!< Pointer to parameter of the current 
                                      executed rev up phase.*/
  int16_t hDirection;            /*!< If it is "1" the programmed revup sequence 
                                      is performed. If it is "-1" the revup 
                                      sequence is performed with opposite values 
                                      of targets (speed, torque).*/
  uint8_t bStageCnt;             /*!< It counts the rev up stages that have been
                                      already started.
                                      NOTE: The rev up stage are zero-based 
                                      indexed so that the fist stage is the 
                                      number zero.*/
  
#ifdef RUC_ALLOWS_TUNING
  RUCPhasesData_t ParamsData[MAX_PHASE_NUMBER]; 
                                 /*!< Rev up phases data. It is initilized equal
                                      to rev up phases parameters but can be
                                      changed in run-time using tuning methods.
                                      */
  uint8_t bPhaseNbr;             /*!< Number of phases relative to the 
                                      programmed rev up sequence.*/
#endif
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef RevupCtrlParams_t Params_t, *pParams_t;

/** 
  * @brief  Private RevupCtrl class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
}_CRUC_t, *_CRUC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__REVUPCTRLPRIVATE_H*/

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
