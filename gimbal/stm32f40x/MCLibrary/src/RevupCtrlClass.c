/**
  ******************************************************************************
  * @file    RevupCtrlClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains interface of RevupCtrl class
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
#include "RevupCtrlClass.h"
#include "RevupCtrlPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CRUC_t RUCpool[MAX_RUC_NUM];
  unsigned char RUC_Allocated = 0u;
#endif
  
#define CLASS_VARS   &((_CRUC)this)->Vars_str
#define CLASS_PARAMS  ((_CRUC)this)->pParams_str
  
/**
  * @brief  Creates an object of the class RevupCtrl
  * @param  pRevupCtrlParams pointer to an RevupCtrl parameters structure
  * @retval CRUC new instance of RevupCtrl object
  */
CRUC RUC_NewObject(pRevupCtrlParams_t pRevupCtrlParams)
{
  _CRUC _oRUC;
  
  #ifdef MC_CLASS_DYNAMIC
    _oRUC = (_CRUC)calloc(1u,sizeof(_CRUC_t));
  #else
    if (RUC_Allocated  < MAX_RUC_NUM)
    {
      _oRUC = &RUCpool[RUC_Allocated++];
    }
    else
    {
      _oRUC = MC_NULL;
    }
  #endif
  
  _oRUC->pParams_str = (pParams_t)pRevupCtrlParams;
  
  return ((CRUC)_oRUC);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the
  *         speed and torque controller and the virtual speed sensor objects to
  *         be used by rev up controller.
  * @param  this related object of class CSTC.
  * @param  oSTC the speed and torque controller used by the RUC
  * @param  oVSS the virtual speed sensor used by the RUC
  * @retval none.
  */
void RUC_Init(CRUC this, CSTC oSTC, CVSS_SPD oVSS)
{
  pVars_t pVars = CLASS_VARS;
#ifdef RUC_ALLOWS_TUNING
  pParams_t pParams = CLASS_PARAMS;
  pRUCPhasesParams_t pRUCPhasesParam = pParams->pPhaseParam;
  uint8_t bPhase = 0u;
#endif
  
  pVars->oSTC = oSTC;
  pVars->oVSS = oVSS;
  
#ifdef RUC_ALLOWS_TUNING
  while ((pRUCPhasesParam != MC_NULL) && (bPhase < MAX_PHASE_NUMBER))
  {
    pVars->ParamsData[bPhase].hDurationms = pRUCPhasesParam->hDurationms;
    pVars->ParamsData[bPhase].hFinalMecSpeed01Hz = 
      pRUCPhasesParam->hFinalMecSpeed01Hz;
    pVars->ParamsData[bPhase].hFinalTorque = pRUCPhasesParam->hFinalTorque;
    pVars->ParamsData[bPhase].pNext = &(pVars->ParamsData[bPhase+1u]);
    bPhase++;
    pRUCPhasesParam = pRUCPhasesParam->pNext;
  }
  pVars->ParamsData[bPhase-1u].pNext = MC_NULL;
  pVars->bPhaseNbr = bPhase;
#endif
}

/**
  * @brief  It should be called before each motor restart. It initialize
  *         internal state of RUC sets first commands to VSS and STC and calls 
  *         the Clear method of VSS.
  *         It also sets the Speed and Torque controller in TORQUE mode.
  * @param  this related object of class CSTC.
  * @param  hMotorDirection If it is "1" the programmed revup sequence is 
  *         performed. If it is "-1" the revup sequence is performed with
  *         opposite values of targets (speed, torque).
  * @retval none.
  */
void RUC_Clear(CRUC this, int16_t hMotorDirection)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  CVSS_SPD oVSS = pVars->oVSS;
  CSTC oSTC = pVars->oSTC;
#ifndef RUC_ALLOWS_TUNING
  pRUCPhasesParams_t pPhaseParams = pParams->pPhaseParam;
#else
  pRUCPhasesData_t pPhaseParams = pVars->ParamsData;
#endif
  
  pVars->hDirection = hMotorDirection;
  
  /* Calls the clear method of VSS.*/
  SPD_Clear((CSPD)oVSS);
  
  /* Sets the STC in torque mode.*/
  STC_SetControlMode(oSTC, STC_TORQUE_MODE);
  
  /* Sets the mechanical starting angle of VSS.*/
  SPD_SetMecAngle((CSPD)oVSS,pParams->hStartingMecAngle * hMotorDirection);
  
  /* Sets to zero the starting torque of STC */
  STC_ExecRamp(oSTC,0,0u);
    
  /* Gives the first command to STC and VSS.*/
  STC_ExecRamp(oSTC,pPhaseParams->hFinalTorque * hMotorDirection,
                    pPhaseParams->hDurationms);
  
  VSPD_SetMecAcceleration((CSPD)oVSS,pPhaseParams->hFinalMecSpeed01Hz * hMotorDirection,
                                     pPhaseParams->hDurationms);
  
  /* Compute hPhaseRemainingTicks.*/
  pVars->hPhaseRemainingTicks = 
    (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
    (uint32_t)pParams->hRUCFrequencyHz) / 1000u);
  pVars->hPhaseRemainingTicks++;
  
  /*Set the next phases parameter pointer.*/
  pVars->pPhaseParams = pPhaseParams->pNext;
  
  /*Initializes the rev up stages counter.*/
  pVars->bStageCnt = 0u;
}

/**
  * @brief  It clocks the rev up controller and must be called with a frequency
  *         equal to the one set in the parameters hRUCFrequencyHz. Calling this
  *         method the rev up controller perform the programmed sequence.
  *         Note: STC and VSS aren’t clocked by RUC_Exec.
  * @param  this related object of class CRUC.
  * @retval bool It returns FALSE when the programmed rev up has been completed. 
  */
bool RUC_Exec(CRUC this)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  pRUCPhasesParams_t pPhaseParams = pVars->pPhaseParams;
  bool retVal=TRUE;
  
  if (pVars->hPhaseRemainingTicks > 0u)
  {
    /* Decrease the hPhaseRemainingTicks.*/
    pVars->hPhaseRemainingTicks--;
  }
  if (pVars->hPhaseRemainingTicks == 0u)
  {
    if(pVars->pPhaseParams!= MC_NULL)
    {
      /* If it becomes zero the current phase has been completed.*/        
      /* Gives the next command to STC and VSS.*/
      STC_ExecRamp(pVars->oSTC,pPhaseParams->hFinalTorque * pVars->hDirection,
                   pPhaseParams->hDurationms);
      
      VSPD_SetMecAcceleration((CSPD)pVars->oVSS,
                              pPhaseParams->hFinalMecSpeed01Hz * pVars->hDirection,
                              pPhaseParams->hDurationms);
      
      /* Compute hPhaseRemainingTicks.*/
      pVars->hPhaseRemainingTicks = 
        (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                    (uint32_t)pParams->hRUCFrequencyHz) / 1000u);
      pVars->hPhaseRemainingTicks++;
      
      /*Set the next phases parameter pointer.*/
      pVars->pPhaseParams = pPhaseParams->pNext;
      
      /*Increases the rev up stages counter.*/
      pVars->bStageCnt ++;
    }
    else
    {
      retVal =FALSE;
    }
  }
  return retVal;
}

/**
  * @brief  It is used to modify the default value of duration of a specific 
  *         rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hDurationms is the new value of duration for that phase.
  * @retval none. 
  */
void RUC_SetPhaseDurationms(CRUC this, uint8_t bPhase, uint16_t hDurationms)
{
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;

  pVars->ParamsData[bPhase].hDurationms = hDurationms;
#endif
}

/**
  * @brief  It is used to modify the default value of mechanical speed at the
  *         end of a specific rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hFinalMecSpeed01Hz is the new value of mechanical speed at the end
  *         of that phase expressed in 0.1Hz.
  * @retval none. 
  */
void RUC_SetPhaseFinalMecSpeed01Hz(CRUC this, uint8_t bPhase, 
                                   int16_t hFinalMecSpeed01Hz)
{
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  pVars->ParamsData[bPhase].hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
#endif
}

/**
  * @brief  It is used to modify the default value of motor torque at the end of
  *         a specific rev up phase.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hFinalTorque is the new value of motor torque at the end of that
  *         phase. This value represents actually the Iq current expressed in
  *         digit.
  * @retval none. 
  */
void RUC_SetPhaseFinalTorque(CRUC this, uint8_t bPhase, int16_t hFinalTorque)
{
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  pVars->ParamsData[bPhase].hFinalTorque = hFinalTorque;
#endif
}

/**
  * @brief  It is used to read the current value of duration of a specific rev
  *         up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval uint16_t The current value of duration for that phase expressed in
  *         milliseconds. 
  */
uint16_t RUC_GetPhaseDurationms(CRUC this, uint8_t bPhase)
{
  uint16_t hRetVal = 0u;
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  hRetVal = pVars->ParamsData[bPhase].hDurationms;
#endif
  return hRetVal;
}

/**
  * @brief  It is used to read the current value of mechanical speed at the end
  *         of a specific rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval int16_t The current value of mechanical speed at the end of that
  *         phase expressed in 0.1Hz. 
  */
int16_t RUC_GetPhaseFinalMecSpeed01Hz(CRUC this, uint8_t bPhase)
{
  int16_t hRetVal = 0;
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  hRetVal = pVars->ParamsData[bPhase].hFinalMecSpeed01Hz;
#endif
  return hRetVal;
}

/**
  * @brief  It is used to read the current value of motor torque at the end of a
  *         specific rev up phase.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval int16_t The current value of motor torque at the end of that phase.
  *         This value represents actually the Iq current expressed in digit.
  */
int16_t RUC_GetPhaseFinalTorque(CRUC this, uint8_t bPhase)
{
  int16_t hRetVal = 0;
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  hRetVal = pVars->ParamsData[bPhase].hFinalTorque;
#endif
  return hRetVal;
}

/**
  * @brief  It is used to get information about the number of phases relative to 
  *         the programmed rev up.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @retval uint8_t The number of phases relative to the programmed rev up.
  */
uint8_t RUC_GetNumberOfPhases(CRUC this)
{
  uint8_t hRetVal = 0u;
#ifdef RUC_ALLOWS_TUNING
  pVars_t pVars = CLASS_VARS;
  
  hRetVal = pVars->bPhaseNbr;
#endif
  return hRetVal;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It is used to know if the programmed "first acceleration stage" has
  *         been reached inside the rev up sequence. Is intended that the stages
  *         previous to this are reserved for alignments. When is reached the
  *         first stage of acceleration the observer should be cleared once.
  *         NOTE: The rev up stage are zero-based indexed so that the fist stage 
  *         is the number zero.
  * @param  this related object of class CRUC.
  * @retval bool It returns TRUE if the first acceleration stage has been 
  *         reached and FALSE otherwise.
  */
bool RUC_FirstAccelerationStageReached(CRUC this)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  bool retVal = FALSE;
  
  if (pVars->bStageCnt >= pParams->bFirstAccelerationStage)
  {
    retVal = TRUE;
  }
  return retVal;
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
