/**
  ******************************************************************************
  * @file    CrossCheck.h 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.0.0
  * @date    28-May-2014 10:45
  * @brief   This file performs the coherency test between control stage 
  *			 parameters values and the compiler options.
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
#ifndef __CROSSCHECK_H
#define __CROSSCHECK_H

#include "Control stage parameters.h"
#include "Drive parameters.h"

/* Backward compatibility */
#if   defined(STM32F103x_LD)
#define STM32PERFORMANCELD
#elif defined(STM32F103x_MD)
#define STM32PERFORMANCEMD
#elif defined(STM32F103x_HD)
#define STM32PERFORMANCEHD
#elif defined(STM32F100x_LD)
#define STM32VALUELD
#elif defined(STM32F100x_MD)
#define STM32VALUEMD
#elif defined(STM32F2xx)
#define CS_STM32F2xx
#elif defined(STM32F4xx)
#define CS_STM32F4xx
#elif (defined(STM32F05xx) || defined(STM32F051x) || defined(STM32F050x) || defined(STM32F030x))
#define CS_STM32F0xx
#if (defined(STM32F05xx))
#define STM32F051x
#endif
#elif defined(STM32F30x)
#define CS_STM32F30x /* Legacy */
#define STM32F303xC
#elif  (defined(STM32F303xB) || defined(STM32F303xC) || defined(STM32F302xB) || defined(STM32F302xC))
#define CS_STM32F30x
#endif

#if (defined(PROJECT_CHK))
  #define ENABLE   1
  #define DISABLE  0
  
  #if ((defined(STM32PERFORMANCELD) || defined(STM32VALUELD)) && (LCD_JOYSTICK_FUNCTIONALITY == ENABLE))
    #error "Invalid configuration: MCU definition in Control stage parameters.h, LD devices not compatible with LCD manager"
  #endif
  
  #if ((defined USE_STM3210E_EVAL) && ((LCD_JOYSTICK_FUNCTIONALITY == ENABLE) && (SW_OV_CURRENT_PROT_ENABLING == ENABLE)))
    #error "In the STM3210E-EVAL, PA6 is shared between TIM8 BKIN and SPI1 MISO, which means that it’s not possible to enable both Overcurrent protection and LCD UI."
  #endif
  
  #undef ENABLE
  #undef DISABLE

  #if (defined(STM32PERFORMANCEMD) && defined(STM32F10X_MD))
  #elif (defined (STM32PERFORMANCELD) && (defined(STM32F10X_LD)||(STM32F10X_MD)))
  #elif (defined(STM32VALUEMD) && defined(STM32F10X_MD_VL))
  #elif (defined(STM32VALUELD) && (defined(STM32F10X_LD_VL)||(STM32F10X_MD_VL)))
  #elif (defined(STM32PERFORMANCEHD) && defined(STM32F10X_HD))
  #elif (defined(CS_STM32F2xx) && defined(STM32F2XX))
  #elif (defined(CS_STM32F4xx) && defined(STM32F40XX))
  #elif (defined(CS_STM32F0xx) && defined(STM32F0XX))
  #elif ((defined(STM32F303xB) || (defined(STM32F303xC))) && defined(STM32F303X))
  #elif ((defined(STM32F302xB) || (defined(STM32F302xC))) && defined(STM32F302X))
  #else
    #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board. Please use the ST Motor Control Workbench to configure the firmware according your system."
  #endif
  
  #if (((defined STM32VALUEMD) || (defined STM32VALUELD)) && !defined(CPU_CLK_24_MHZ))
  #error "Invalid configuration: MCU frequency definition in Control stage parameters.h not compatible with selected MCU"
  #endif

  #if (defined(STM32F050x) || defined(STM32F030x))
    #warning "Remember to configure in user project settings the target MCU device according to the one setled in Control stage parameters.h. Once done, comment CrossCheck.h line 91 to avoid to show this messagge again."
  #endif

  #if (PWM_FREQUENCY < 4500)
    #error "Invalid configuration: PWM frequency settled in Drive parameters.h is not supported by the firmware"
  #endif

#endif

#if (defined(MC_APPLICATION_CHK))
  #if (defined(STM32PERFORMANCEMD) && defined(STM32F10X_MD))
  #elif (defined (STM32PERFORMANCELD) && (defined(STM32F10X_LD)||(STM32F10X_MD)))
  #elif (defined(STM32VALUEMD) && defined(STM32F10X_MD_VL))
  #elif (defined(STM32VALUELD) && (defined(STM32F10X_LD_VL)||(STM32F10X_MD_VL)))
  #elif (defined(STM32PERFORMANCEHD) && defined(STM32F10X_HD))
  #elif ((defined(STM32PERFORMANCEMD) || defined (STM32PERFORMANCELD) || defined(STM32VALUEMD) || defined(STM32VALUELD) || defined(STM32PERFORMANCEHD)) && defined(STM32F10X_HD))
  #elif (defined(CS_STM32F2xx) && defined(STM32F2XX))
  #elif (defined(CS_STM32F4xx) && defined(STM32F40XX))
  #elif (defined(CS_STM32F0xx) && defined(STM32F0XX))
  #elif (defined(CS_STM32F30x) && defined(STM32F30X))
  #else
    #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board. Please use the ST Motor Control Workbench to configure the firmware according your system."
  #endif
  
  #if (defined(DUALDRIVE) && (!defined(STM32F10X_HD) && !defined(STM32F40XX) && !defined(STM32F2XX) && !defined(STM32F303xB) && !defined(STM32F303xC)))
  #error "Invalid configuration: DUALDRIVE definition in Drive parameters.h is not supported by selected MCU"
  #endif

  #define ENABLE   1
  #define DISABLE  0

  #if ((OPEN_LOOP_FOC == ENABLE)&&(!defined(STATE_OBSERVER_PLL))&&(!defined(STATE_OBSERVER_CORDIC)))
    #error "Invalid configuration: OPEN_LOOP configuration shall be enabled with Sensor-less enabled in Drive parameters.h"
  #endif

#if defined(DUALDRIVE)
  #include "Drive parameters motor 2.h"

  #if ((OPEN_LOOP_FOC2 == ENABLE)&&(!defined(STATE_OBSERVER_PLL2))&&(!defined(STATE_OBSERVER_CORDIC2)))
    #error "Invalid configuration: OPEN_LOOP configuration shall be enabled with Sensor-less enabled in Drive parameters motor 2.h"
  #endif
#endif

  #undef ENABLE
  #undef DISABLE

#endif
    
#if (defined(LCD_CHK))
  #if (defined(STM32PERFORMANCELD) || defined(STM32VALUELD) || defined(STM32F10X_LD) || defined(STM32F10X_LD_VL))
  #error "Invalid configuration: MCU definition in Control stage parameters.h, LD devices not compatible with LCD manager"
  #endif

  #if (defined(STM32F10X_MD) && !defined(STM32PERFORMANCEMD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(STM32F10X_MD_VL) && !defined(STM32VALUEMD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(STM32F10X_HD) && !defined(STM32PERFORMANCEHD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(DUALDRIVE) && (!defined(STM32F10X_HD) && !defined(STM32F40XX) && !defined(STM32F2XX) && !defined(STM32F303xB) && !defined(STM32F303xC)))
  #error "Invalid configuration: DUALDRIVE definition in Drive parameters.h is not supported by selected MCU"
  #endif

  #if (defined(STM32VALUE) && !defined(CPU_CLK_24_MHZ))
  #error "Invalid configuration: MCU frequency definition in Control stage parameters.h not compatible with selected MCU"
  #endif

  #if (defined(STM32VALUE) && defined(PFC_ENABLED))
  #error "Invalid configuration: PFC can't be enabled using selected MCU"
  #endif

  #if (defined(STM32F0XX) && !defined(LCD_REDUCED_FUNCTIONS))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS must be defined with selected MCU"
  #endif

  #if (defined(LCD_REDUCED_FUNCTIONS) && defined(DUALDRIVE))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS not compatible with DUALDRIVE"
  #endif

  #if (defined(LCD_REDUCED_FUNCTIONS) && defined(PFC_ENABLED))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS not compatible with PFC_ENABLED"
  #endif
#endif

#endif /* __TIMEBASE_H */

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
