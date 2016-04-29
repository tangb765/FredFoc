/**
  ******************************************************************************
  * @file    Parameters conversion.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#if (defined(STM32F10X_MD) || defined (STM32F10X_LD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_LD_VL) || defined(STM32F10X_HD))
  #include "Parameters conversion_F10x.h"
#elif defined(STM32F2XX)
  #include "Parameters conversion_F2xx.h"
#elif defined(STM32F30X)
  #include "Parameters conversion_F30x.h"
#elif defined(STM32F40XX)
  #include "Parameters conversion_F4xx.h"
#elif defined(STM32F0XX)
  #include "Parameters conversion_F0xx.h"
#endif

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
