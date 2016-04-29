/**
  ******************************************************************************
  * @file    VirtualSpeedSensor_SpeednPosFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains private impelementation of VirtualSpeedSensor class      
  ******************************************************************************
  * @attention
  *
  * <h2><center>?0?8 COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#include "SpeednPosFdbkClass.h"
#include "SpeednPosFdbkPrivate.h"
#include "VirtualSpeedSensor_SpeednPosFdbkClass.h"
#include "VirtualSpeedSensor_SpeednPosFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

/* Private Defines -----------------------------------------------------------*/
#define DCLASS_PARAM ((_DCVSS_SPD)(((_CSPD) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCVSS_SPD)(((_CSPD) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CSPD)this)->Vars_str)
#define  CLASS_PARAM (((_CSPD)this)->pParams_str)

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCVSS_SPD_t VSS_SPDpool[MAX_VSS_SPD_NUM];
	unsigned char VSS_SPD_Allocated = 0u;
#endif

static void VSS_Init(CSPD this);
static void VSS_Clear(CSPD this);
static int16_t VSS_CalcElAngle(CSPD this, void *pInputVars_str, int32_t Aligned_num, uint8_t bMotorNbr);
static bool VSS_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz);
static int32_t VSS_SetMecAngle(CSPD this, int16_t hMecAngle);

/**
  * @brief  Creates an object of the class VirtualSpeedSensor
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pVirtualSpeedSensorParams pointer to an VirtualSpeedSensor parameters structure
  * @retval CVSS_SPD new instance of VirtualSpeedSensor object
  */
CVSS_SPD VSS_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pVirtualSpeedSensorParams_t pVirtualSpeedSensorParams)
{
	_CSPD _oSpeednPosFdbk;
	_DCVSS_SPD _oVirtualSpeedSensor;

	_oSpeednPosFdbk = (_CSPD)SPD_NewObject(pSpeednPosFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oVirtualSpeedSensor = (_DCVSS_SPD)calloc(1u,sizeof(_DCVSS_SPD_t));
	#else
		if (VSS_SPD_Allocated  < MAX_VSS_SPD_NUM)
		{
			_oVirtualSpeedSensor = &VSS_SPDpool[VSS_SPD_Allocated++];
		}
		else
		{
			_oVirtualSpeedSensor = MC_NULL;
		}
	#endif
  
	_oVirtualSpeedSensor->pDParams_str = pVirtualSpeedSensorParams;
	_oSpeednPosFdbk->DerivedClass = (void*)_oVirtualSpeedSensor;
  
  _oSpeednPosFdbk->Methods_str.pSPD_Init = &VSS_Init;
  _oSpeednPosFdbk->Methods_str.pSPD_Clear = &VSS_Clear;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAngle = &VSS_CalcElAngle;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAvrgMecSpeed01Hz = 
    &VSS_CalcAvrgMecSpeed01Hz;
  _oSpeednPosFdbk->Methods_str.pSPD_SetMecAngle = &VSS_SetMecAngle;

	return ((CVSS_SPD)_oSpeednPosFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_VirtualSpeedSensor
  * @{
  */

/** @defgroup VirtualSpeedSensor_class_private_methods VirtualSpeedSensor class private methods
* @{
*/

/**
  * @brief  Software initialization of VSS object.
  * @param  this related object of class CSPD
  * @param  InputVars_str not used by VSS.
  * @retval none
  */
static void VSS_Init(CSPD this)
{
  VSS_Clear(this);
}

/**
* @brief  Software initializzation of VSS object to be performed at each restart
*         of the motor.
* @param  this related object of class CSPD
* @retval none
*/
static void VSS_Clear(CSPD this)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  pVars_t pVars_str = CLASS_VARS;
  
  pVars_str->hElAngle = 0;
  pVars_str->hMecAngle = 0;
  pVars_str->hAvrMecSpeed01Hz = 0;
  pVars_str->hElSpeedDpp = 0;
  pVars_str->hMecAccel01HzP = 0;
  pVars_str->bSpeedErrorNumber = 0u;
  
  pDVars_str->wElAccDppP32 = 0;
  pDVars_str->wElSpeedDpp32 = 0;
  pDVars_str->hRemainingStep = 0u;
  pDVars_str->hElAngleAccu = 0;
  
  pDVars_str->bTransitionStarted = FALSE;
  pDVars_str->bTransitionEnded = FALSE;
  pDVars_str->hTransitionRemainingSteps = DCLASS_PARAM->hTransitionSteps;
  pDVars_str->bTransitionLocked = FALSE;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Update the rotor electrical angle integrating the last setled 
*         instantaneous electrical speed express in dpp.
* @param  this related object of class CSPD.
* @retval int16_t Measured electrical angle in s16degree format.
*/
static int16_t VSS_CalcElAngle(CSPD this, void *pInputVars_str, int32_t Aligned_num, uint8_t bMotorNbr)
{
  pVars_t pVars_str = CLASS_VARS;
  pDVars_t pDVars_str = DCLASS_VARS;  
  pParams_t pParams_str = CLASS_PARAM;
  int16_t hRetAngle;
  int16_t hAngleDiff;
  int16_t hAngleCorr;
  int32_t wAux;
  int16_t hSignCorr = 1;
  
  pDVars_str->hElAngleAccu += pVars_str->hElSpeedDpp;
  pVars_str->hMecAngle += pVars_str->hElSpeedDpp / 
    (int16_t)pParams_str->bElToMecRatio;
  
  if (pDVars_str->bTransitionStarted == TRUE)
  {    
    if (pDVars_str->hTransitionRemainingSteps == 0)
    {
      hRetAngle = *(int16_t*)pInputVars_str;
      pDVars_str->bTransitionEnded = TRUE;
      pVars_str->bSpeedErrorNumber = 0u;
    }
    else
    {
      pDVars_str->hTransitionRemainingSteps--;
      
      if (pVars_str->hElSpeedDpp >= 0)
      {
        hAngleDiff = *(int16_t*)pInputVars_str - pDVars_str->hElAngleAccu;
      }
      else
      {
        hAngleDiff = pDVars_str->hElAngleAccu - *(int16_t*)pInputVars_str;
        hSignCorr = -1;
      }        
      
      wAux = (int32_t)hAngleDiff * pDVars_str->hTransitionRemainingSteps;
      hAngleCorr = (int16_t)(wAux/DCLASS_PARAM->hTransitionSteps);
      
      hAngleCorr *= hSignCorr;
      
      if (hAngleDiff >= 0)
      {
        pDVars_str->bTransitionLocked = TRUE;
        hRetAngle = *(int16_t*)pInputVars_str - hAngleCorr;
      }
      else
      {
        if (pDVars_str->bTransitionLocked == FALSE)
        {
          hRetAngle = pDVars_str->hElAngleAccu;
        }
        else
        {          
          hRetAngle = *(int16_t*)pInputVars_str + hAngleCorr;
        }
      }
    }
  }
  else
  {
    hRetAngle = pDVars_str->hElAngleAccu;
  }

  pVars_str->hElAngle = hRetAngle;
  return hRetAngle;
}

/**
  * @brief  This method must be called with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express 
  *         in dpp considering the measurement frequency) in order to provide it
  *         to SPD_CalcElAngle function and SPD_GetElAngle. 
  *         Then compute store and return - through parameter 
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then return the reliability state of the sensor (allways TRUE).
  * @param  this related object of class CSPD
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  */
static bool VSS_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  pVars_t pVars_str = CLASS_VARS;
  pParams_t pParams_str = CLASS_PARAM;
  
  if (pDVars_str->hRemainingStep > 1u)
  {
    pDVars_str->wElSpeedDpp32 += pDVars_str->wElAccDppP32;
    pVars_str->hElSpeedDpp = (int16_t)(pDVars_str->wElSpeedDpp32 / 65536);
  
    /* Converto el_dpp to Mec01Hz */
    *hMecSpeed01Hz = ((pVars_str->hElSpeedDpp * 
                       (int16_t)pParams_str->hMeasurementFrequency * 10)/
                      (65536 * (int16_t)pParams_str->bElToMecRatio));
    
    pVars_str->hAvrMecSpeed01Hz = *hMecSpeed01Hz;
    
    pDVars_str->hRemainingStep--;
  }
  else if (pDVars_str->hRemainingStep == 1u)
  {
    *hMecSpeed01Hz = pDVars_str->hFinalMecSpeed01Hz;
    
    pVars_str->hAvrMecSpeed01Hz = *hMecSpeed01Hz;
    
    pVars_str->hElSpeedDpp = (int16_t)(((int32_t)(*hMecSpeed01Hz) * 
                   (int32_t)65536)/
                   ((int32_t)10 * (int32_t)pParams_str->hMeasurementFrequency)); 
    
    pVars_str->hElSpeedDpp *= (int16_t)(pParams_str->bElToMecRatio);
    
    pDVars_str->hRemainingStep = 0u;
  }
  else
  {
    *hMecSpeed01Hz = pVars_str->hAvrMecSpeed01Hz;
  }
    
  return (pDVars_str->bTransitionEnded);
}

/**
  * @brief  It is used to set istantaneous information on VSS mechanical and
  *         electrical angle.
  * @param  this related object of class CSPD
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
static int32_t VSS_SetMecAngle(CSPD this, int16_t hMecAngle)
{
  pVars_t pVars_str = CLASS_VARS;
  pParams_t pParams_str = CLASS_PARAM; 
  pDVars_t pDVars_str = DCLASS_VARS;
  
  pDVars_str->hElAngleAccu = hMecAngle;
  pVars_str->hMecAngle = pDVars_str->hElAngleAccu/(int16_t)pParams_str->bElToMecRatio;
  pVars_str->hElAngle = hMecAngle;
	return 0;
}


/**
  * @brief  Set the mechanical acceleration of virtual sensor. This acceleration
            is defined starting from current mechanical speed, final mechanical
            speed expressed in 0.1Hz and duration expressed in milliseconds.
  * @param  this related object of class CSTC.
  * @param  hFinalMecSpeed01Hz mechanical speed expressed in 0.1Hz assumed by 
            the virtual sensor at the end of the duration.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply 
            instantaneous the final speed. 
  * @retval none
  */
void  VSPD_SetMecAcceleration(CSPD this, int16_t  hFinalMecSpeed01Hz, 
                              uint16_t hDurationms)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  pVars_t pVars_str = CLASS_VARS;
  pParams_t pParams_str = CLASS_PARAM;
  
  uint16_t hNbrStep;
  int16_t hCurrentMecSpeedDpp;
  int32_t wMecAccDppP32;
  int16_t hFinalMecSpeedDpp;
  
  if (pDVars_str->bTransitionStarted == FALSE)
  {    
    if (hDurationms == 0u)
    {      
      pVars_str->hAvrMecSpeed01Hz = hFinalMecSpeed01Hz;
      
      pVars_str->hElSpeedDpp = (int16_t)(((int32_t)(hFinalMecSpeed01Hz) * 
                                          (int32_t)65536)/
                                         ((int32_t)10 * (int32_t)pParams_str->hMeasurementFrequency)); 
      
      pVars_str->hElSpeedDpp *= (int16_t)(pParams_str->bElToMecRatio);
      
      pDVars_str->hRemainingStep = 0u;
      
      pDVars_str->hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
    }
    else
    {
      hNbrStep = (uint16_t)(((uint32_t)hDurationms * 
                             (uint32_t)pDParams_str->hSpeedSamplingFreqHz) / 
                            1000u);
      
      hNbrStep++;
      
      pDVars_str->hRemainingStep = hNbrStep;
      
      hCurrentMecSpeedDpp = pVars_str->hElSpeedDpp / 
        (int16_t)pParams_str->bElToMecRatio;
      
      hFinalMecSpeedDpp = (int16_t)(((int32_t)hFinalMecSpeed01Hz * (int32_t)65536)/
                                    ((int32_t)10 * (int32_t)pParams_str->hMeasurementFrequency)); 
      
      wMecAccDppP32 = (((int32_t)hFinalMecSpeedDpp - (int32_t)hCurrentMecSpeedDpp) * 
                       (int32_t)65536) /
        (int32_t)hNbrStep;
      
      pDVars_str->wElAccDppP32 = wMecAccDppP32 * (int16_t)pParams_str->bElToMecRatio;
      
      pDVars_str->hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
      
      pDVars_str->wElSpeedDpp32 = (int32_t)pVars_str->hElSpeedDpp * (int32_t)65536;
    }
  }
}

/**
  * @brief  Get the final speed of last setled ramp of virtual sensor expressed 
            in 0.1Hz.
  * @param  this related object of class CSTC.
  * @param  hFinalMecSpeed01Hz mechanical speed expressed in 0.1Hz assumed by 
            the virtual sensor at the end of the duration.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply 
            instantaneous the final speed. 
  * @retval none
  */
int16_t  VSPD_GetLastRampFinalSpeed(CSPD this)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  return pDVars_str->hFinalMecSpeed01Hz;
}

/**
  * @brief  Set the command to Start the transition phase from CVSS_SPD to other CSPD.
            Transition is to be considered ended when Sensor information is
            declared 'Reliable' or if function returned value is FALSE
  * @param  this related object of class CSPD.
  * @param  bool TRUE to Start the transition phase, FALSE has no effect
  * @retval bool TRUE if Transition phase is enabled (started or not), FALSE if
            transition has been triggered but it's actually disabled
            (parameter hTransitionSteps = 0)
  */
bool VSPD_SetStartTransition(CSPD this, bool bCommand)
{
  bool bAux = TRUE;
  if (bCommand == TRUE)
  {
    (DCLASS_VARS)->bTransitionStarted = TRUE;
    
    if (DCLASS_PARAM->hTransitionSteps == 0)
    {
      (DCLASS_VARS)->bTransitionEnded = TRUE;
      (CLASS_VARS)->bSpeedErrorNumber = 0u;
      bAux = FALSE;
    }
  }
  return bAux;
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
