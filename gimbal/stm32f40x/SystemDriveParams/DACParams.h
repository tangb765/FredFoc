/**
  ******************************************************************************
  * @file    DACParams.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file contains DAC constant parameters definition
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
#ifndef __DAC_PARAMS_H
#define __DAC_PARAMS_H
#include "Parameters conversion.h"

#if defined(STM32F30X) /* To be fixed adding dummy defines to avoid conditional compilation (if required) */

const DAC_F30XParams_t DAC_F30XParams_str = 
{
  DEBUG_DAC_CH1,          /*!< Set to ENABLE to assign the channel 1 to the DAC object 
                        otherwise set DISABLE */ 
  DEBUG_DAC_CH2           /*!< Set to ENABLE to assign the channel 2 to the DAC object 
                        otherwise set DISABLE */
};

#endif // STM32F30X

#endif /* __DAC_PARAMS_H */

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
