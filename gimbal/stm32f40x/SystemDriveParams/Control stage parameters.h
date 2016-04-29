/**
  ******************************************************************************
  * @file    Control stage parameters.h
  * @author  STMCWB ver.4.2.0.15408
  * @version 4.1.0
  * @date    2016-23-03 17:15:22
  * @project STMF4_SINGLE.stmcx
  * @path    G:\software\Gimbal_2016_0323\rt-thread\bsp\stm32f40x
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *                 library v4.1.0
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
#ifndef __CONTROL_STAGE_PARAMETERS_H
#define __CONTROL_STAGE_PARAMETERS_H

/***************************** MCU SELECTION SECTION **************************/
#define STM32F4xx  /* STM32F103x_HD, STM32F103x_MD, STM32F103x_LD, STM32F100x_LD, 
							 STM32F100x_MD, STM32F4xx, STM32F2xx, STM32F05xx,
                             STM32F303xB, STM32F303xC, STM32F302xB, STM32F302xC*/
#define PACKAGE_SELECTION     NOT_USED0   /*VFQFPN36, VFQFPN48, LQFP48, LQFP64,  
                                                 LQFP100, LQFP144, WLCSP64, LFBGA100, 
                                                 LFBGA144, BGA100, BGA64, TFBGA64*/

/***************************** MCU supply voltage *****************************/
#define MCU_SUPPLY_VOLTAGE    3.30

/***************************** CLOCK SETTINGS SECTION *************************/
#define CLOCK_SOURCE          EXTERNAL  /* EXTERNAL or INTERNAL */

#define CPU_CLK_168_MHZ


/* ext. clock frequency */
#define EXT_CLK_8_MHZ

/* Maximum modulation index */
#define MAX_MODULATION_93_PER_CENT
                         
/************************ DIGITAL I/O DEFINITION SECTION  *********************/
/* PWM Timer section */
#define PWM_TIMER_SELECTION               PWM_TIM1 /* PWM_TIM1 or PWM_TIM8 */ 
#define PWM_TIMER_REMAPPING               PWM_NO_REMAP /* PWM_NO_REMAP,  
                                                           PWM_FULL_REMAP, 
                                                           PWM_PARTIAL_REMAP */
#define PHASE_UH_GPIO_PORT                GPIOA                                                                            
#define PHASE_UH_GPIO_PIN                 GPIO_Pin_8                                                                        
#define PHASE_VH_GPIO_PORT                GPIOA                                                                                                                                                    
#define PHASE_VH_GPIO_PIN                 GPIO_Pin_9
#define PHASE_WH_GPIO_PORT                GPIOA                                                                         
#define PHASE_WH_GPIO_PIN                 GPIO_Pin_10
#define PHASE_UL_GPIO_PORT                GPIOB                                                                        
#define PHASE_UL_GPIO_PIN                 GPIO_Pin_2
#define PHASE_VL_GPIO_PORT                GPIOB                                                                
#define PHASE_VL_GPIO_PIN                 GPIO_Pin_2
#define PHASE_WL_GPIO_PORT                GPIOB                                                        
#define PHASE_WL_GPIO_PIN                 GPIO_Pin_2
#define EMERGENCY_STOP_GPIO_PORT          GPIOB
#define EMERGENCY_STOP_GPIO_PIN           GPIO_Pin_12

#define BKIN_MODE                         NONE /* NONE, INT_MODE, EXT_MODE */
#define BKIN2_MODE                        NONE /* NONE, INT_MODE, EXT_MODE */
#define EMERGENCY2_STOP_GPIO_PORT         GPIOB
#define EMERGENCY2_STOP_GPIO_PIN          GPIO_Pin_12
#define PHASE_UH_GPIO_AF                  GPIO_AF_6
#define PHASE_VH_GPIO_AF                  GPIO_AF_6
#define PHASE_WH_GPIO_AF                  GPIO_AF_6
#define PHASE_UL_GPIO_AF                  GPIO_AF_0
#define PHASE_VL_GPIO_AF                  GPIO_AF_0
#define PHASE_WL_GPIO_AF                  GPIO_AF_0
#define BRKIN_GPIO_AF                     GPIO_AF_6
#define BRKIN2_GPIO_AF                    GPIO_AF_2

/* Hall timer section */
#define HALL_TIMER_SELECTION              HALL_TIM4  /* HALL_TIM2...HALL_TIM5 */ 
#define HALL_TIMER_REMAPPING              NO_REMAP_TIM4  /* NO_REMAP, FULL_REMAP, 
                                                            PARTIAL_REMAP */
#define H1_GPIO_PORT                      GPIOD
#define H2_GPIO_PORT                      GPIOD
#define H3_GPIO_PORT                      GPIOB

#define H1_GPIO_PIN                       GPIO_Pin_12
#define H2_GPIO_PIN                       GPIO_Pin_13
#define H3_GPIO_PIN                       GPIO_Pin_8

/* Encoder timer selection */
#define ENC_TIMER_SELECTION               ENC_TIM3  /* ENC_TIM2...HALL_TIM5 */ 
#define ENC_TIMER_REMAPPING               NO_REMAP_TIM3  /* NO_REMAP, FULL_REMAP, 
                                                            PARTIAL_REMAP */
#define ENC_A_GPIO_PORT                   GPIOC
#define ENC_B_GPIO_PORT                   GPIOC
#define ENC_A_GPIO_PIN                    GPIO_Pin_6
#define ENC_B_GPIO_PIN                    GPIO_Pin_7

/* Digital Outputs */
#define R_BRAKE_GPIO_PORT                  GPIOD
#define R_BRAKE_GPIO_PIN                   GPIO_Pin_5
#define OV_CURR_BYPASS_GPIO_PORT           GPIOD        
#define OV_CURR_BYPASS_GPIO_PIN            GPIO_Pin_5
#define INRUSH_CURRLIMIT_GPIO_PORT         GPIOD
#define INRUSH_CURRLIMIT_GPIO_PIN          GPIO_Pin_4

/************************ ANALOG I/O DEFINITION SECTION  *********************/
/** Currents reading  **/
/* Only for three shunt resistors and ICS cases */
#define ADC_1_PERIPH                    ADC1
#define ADC_2_PERIPH                    ADC2
#define PHASE_U_CURR_ADC                ADC1_2
#define PHASE_U_CURR_CHANNEL            ADC_Channel_0
#define PHASE_U_GPIO_PORT               GPIOA
#define PHASE_U_GPIO_PIN                GPIO_Pin_0
#define PHASE_V_CURR_ADC                ADC1_2
#define PHASE_V_CURR_CHANNEL            ADC_Channel_1
#define PHASE_V_GPIO_PORT               GPIOA
#define PHASE_V_GPIO_PIN                GPIO_Pin_1
/* Only for three shunts case */
#define PHASE_W_CURR_ADC                ADC1_2
#define PHASE_W_CURR_CHANNEL            ADC_Channel_2
#define PHASE_W_GPIO_PORT               GPIOA
#define PHASE_W_GPIO_PIN                GPIO_Pin_2       
/* Only for 1 shunt resistor case */
#define ADC_PERIPH                      ADC1
#define PHASE_CURRENTS_CHANNEL          ADC_Channel_0
#define PHASE_CURRENTS_GPIO_PORT        GPIOA
#define PHASE_CURRENTS_GPIO_PIN         GPIO_Pin_0   

/* Common */
#define ADC_AHBPERIPH                   RCC_AHBPeriph_ADC12
#define ADC_CLOCK_WB_FREQ               21
#define ADC_CLOCK_WB_DIV                0
#define CURR_SAMPLING_TIME              15  /*!< Sampling time duration  
                                                           in ADC clock cycles (1 for  
                                                           1.5, 7 for 7.5, ...) */

/** Bus and temperature readings **/
#define REGCONVADC                      ADC1

#define VBUS_ADC                        ADC1_2
#define VBUS_CHANNEL                    ADC_Channel_2
#define VBUS_GPIO_PORT                  GPIOA
#define VBUS_GPIO_PIN                   GPIO_Pin_2
#define VBUS_ADC_SAMPLING_TIME          56

#define TEMP_FDBK_ADC                   ADC1_2
#define TEMP_FDBK_CHANNEL               ADC_Channel_15
#define TEMP_FDBK_GPIO_PORT             GPIOC
#define TEMP_FDBK_GPIO_PIN              GPIO_Pin_5
#define TEMP_ADC_SAMPLING_TIME          28

/* Serial communication */
#define USART_SELECTION                 USE_USART3
#define USART_REMAPPING                 NO_REMAP_USART3
#define USART_TX_GPIO_PORT              GPIOC
#define USART_TX_GPIO_PIN               GPIO_Pin_10
#define USART_RX_GPIO_PORT              GPIOC
#define USART_RX_GPIO_PIN               GPIO_Pin_11
#define USART_SPEED						115200

/* OPAMP Settings */

#define USE_INTERNAL_OPAMP                     DISABLE

#define OPAMP1_SELECTION                       OPAMP_Selection_OPAMP1
#define OPAMP1_INVERTINGINPUT_MODE             INT_MODE
#define OPAMP1_INVERTINGINPUT                  OPAMP1_InvertingInput_PGA
#define OPAMP1_INVERTINGINPUT_GPIO_PORT        GPIOA
#define OPAMP1_INVERTINGINPUT_GPIO_PIN         GPIO_Pin_3
#define OPAMP1_NONINVERTINGINPUT_PHA           OPAMP1_NonInvertingInput_PA1
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT GPIOA
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN  GPIO_Pin_1
#define OPAMP1_NONINVERTINGINPUT_PHB           OPAMP1_NonInvertingInput_PA7
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT GPIOA
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN  GPIO_Pin_7
#define OPAMP1_OUT_GPIO_PORT                   GPIOA
#define OPAMP1_OUT_GPIO_PIN                    GPIO_Pin_2

#define OPAMP2_SELECTION                       OPAMP_Selection_OPAMP2
#define OPAMP2_INVERTINGINPUT_MODE             INT_MODE
#define OPAMP2_INVERTINGINPUT                  OPAMP2_InvertingInput_PGA
#define OPAMP2_INVERTINGINPUT_GPIO_PORT        GPIOC
#define OPAMP2_INVERTINGINPUT_GPIO_PIN         GPIO_Pin_5
#define OPAMP2_NONINVERTINGINPUT_PHA           OPAMP2_NonInvertingInput_PD14
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PORT GPIOD
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PIN  GPIO_Pin_14
#define OPAMP2_NONINVERTINGINPUT_PHB           OPAMP2_NonInvertingInput_PA7
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT GPIOA
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN  GPIO_Pin_7
#define OPAMP2_NONINVERTINGINPUT_PHC           OPAMP2_NonInvertingInput_PD14
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT GPIOD
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN  GPIO_Pin_14
#define OPAMP2_OUT_GPIO_PORT                   GPIOA
#define OPAMP2_OUT_GPIO_PIN                    GPIO_Pin_6

/* Only for 1 shunt resistor case */
#define OPAMP_SELECTION                        OPAMP_Selection_OPAMP1
#define OPAMP_INVERTINGINPUT_MODE              INT_MODE
#define OPAMP_INVERTINGINPUT                   OPAMP1_InvertingInput_PGA
#define OPAMP_INVERTINGINPUT_GPIO_PORT         GPIOA
#define OPAMP_INVERTINGINPUT_GPIO_PIN          GPIO_Pin_3
#define OPAMP_NONINVERTINGINPUT                OPAMP1_NonInvertingInput_PA7
#define OPAMP_NONINVERTINGINPUT_GPIO_PORT      GPIOA
#define OPAMP_NONINVERTINGINPUT_GPIO_PIN       GPIO_Pin_7
#define OPAMP_OUT_GPIO_PORT                    GPIOA
#define OPAMP_OUT_GPIO_PIN                     GPIO_Pin_2

/* OPAMP common settings*/
#define OPAMP_PGAGAIN                          OPAMP_OPAMP_PGAGain_2
#define OPAMP_PGACONNECT                       OPAMP_PGAConnect_No

/* COMP Settings */

#define INTERNAL_OVERCURRENTPROTECTION    DISABLE
#define OCPREF                            23830

#define INTERNAL_OVERVOLTAGEPROTECTION    DISABLE
#define OVPREF                            23830

/* Only for 1 shunt resistor case */
#define OCP_SELECTION                     COMP_Selection_COMP2
#define OCP_INVERTINGINPUT_MODE           INT_MODE
#define OCP_INVERTINGINPUT                COMPX_InvertingInput_VREF
#define OCP_INVERTINGINPUT_GPIO_PORT      GPIOA
#define OCP_INVERTINGINPUT_GPIO_PIN       GPIO_Pin_4
#define OCP_NONINVERTINGINPUT             COMP2_NonInvertingInput_PA7
#define OCP_NONINVERTINGINPUT_GPIO_PORT   GPIOA
#define OCP_NONINVERTINGINPUT_GPIO_PIN    GPIO_Pin_7
#define OCP_OUTPUT_MODE                   INT_MODE
#define OCP_OUTPUT                        COMP_Output_TIM1BKIN2
#define OCP_OUTPUT_GPIO_PORT              GPIOA
#define OCP_OUTPUT_GPIO_PIN               GPIO_Pin_2
#define OCP_OUTPUT_GPIO_AF                GPIO_AF_8
#define OCP_OUTPUTPOL                     COMP_OutputPol_NonInverted

#define OCPA_SELECTION                    COMP_Selection_COMP1
#define OCPA_INVERTINGINPUT_MODE          INT_MODE
#define OCPA_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPA_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPA_INVERTINGINPUT_GPIO_PIN      GPIO_Pin_4
#define OCPA_NONINVERTINGINPUT            COMP1_NonInvertingInput_PA1
#define OCPA_NONINVERTINGINPUT_GPIO_PORT  GPIOA
#define OCPA_NONINVERTINGINPUT_GPIO_PIN   GPIO_Pin_1
#define OCPA_OUTPUT_MODE                  INT_MODE
#define OCPA_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPA_OUTPUT_GPIO_PORT             GPIOA
#define OCPA_OUTPUT_GPIO_PIN              GPIO_Pin_0
#define OCPA_OUTPUT_GPIO_AF               GPIO_AF_8
#define OCPA_OUTPUTPOL                    COMP_OutputPol_NonInverted

#define OCPB_SELECTION                    COMP_Selection_COMP2
#define OCPB_INVERTINGINPUT_MODE          INT_MODE
#define OCPB_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPB_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPB_INVERTINGINPUT_GPIO_PIN      GPIO_Pin_4
#define OCPB_NONINVERTINGINPUT            COMP2_NonInvertingInput_PA7
#define OCPB_NONINVERTINGINPUT_GPIO_PORT  GPIOA
#define OCPB_NONINVERTINGINPUT_GPIO_PIN   GPIO_Pin_7
#define OCPB_OUTPUT_MODE                  INT_MODE
#define OCPB_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPB_OUTPUT_GPIO_PORT             GPIOA
#define OCPB_OUTPUT_GPIO_PIN              GPIO_Pin_2
#define OCPB_OUTPUT_GPIO_AF               GPIO_AF_8
#define OCPB_OUTPUTPOL                    COMP_OutputPol_NonInverted

#define OCPC_SELECTION                    COMP_Selection_COMP3
#define OCPC_INVERTINGINPUT_MODE          INT_MODE
#define OCPC_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPC_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPC_INVERTINGINPUT_GPIO_PIN      GPIO_Pin_4
#define OCPC_NONINVERTINGINPUT            COMP3_NonInvertingInput_PD14
#define OCPC_NONINVERTINGINPUT_GPIO_PORT  GPIOD
#define OCPC_NONINVERTINGINPUT_GPIO_PIN   GPIO_Pin_14
#define OCPC_OUTPUT_MODE                  INT_MODE
#define OCPC_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPC_OUTPUT_GPIO_PORT             GPIOC
#define OCPC_OUTPUT_GPIO_PIN              GPIO_Pin_8
#define OCPC_OUTPUT_GPIO_AF               GPIO_AF_7
#define OCPC_OUTPUTPOL                    COMP_OutputPol_NonInverted
                                                              
#define OVP_SELECTION                     COMP_Selection_COMP7
#define OVP_INVERTINGINPUT_MODE           INT_MODE
#define OVP_INVERTINGINPUT                COMPX_InvertingInput_VREF
#define OVP_INVERTINGINPUT_GPIO_PORT      GPIOA
#define OVP_INVERTINGINPUT_GPIO_PIN       GPIO_Pin_4
#define OVP_NONINVERTINGINPUT             COMP7_NonInvertingInput_PA0
#define OVP_NONINVERTINGINPUT_GPIO_PORT   GPIOA
#define OVP_NONINVERTINGINPUT_GPIO_PIN    GPIO_Pin_0
#define OVP_OUTPUT_MODE                   INT_MODE
#define OVP_OUTPUT                        COMP_Output_TIM1BKIN
#define OVP_OUTPUT_GPIO_PORT              GPIOC
#define OVP_OUTPUT_GPIO_PIN               GPIO_Pin_2
#define OVP_OUTPUT_GPIO_AF                GPIO_AF_3
#define OVP_OUTPUTPOL                     COMP_OutputPol_NonInverted

#define HIGH_SIDE_BRAKE_STATE             TURN_OFF /*!< TURN_OFF, TURN_ON */
#define LOW_SIDE_BRAKE_STATE              TURN_OFF /*!< TURN_OFF, TURN_ON */

#define BKIN1_FILTER                      0
#define BKIN2_FILTER                      0

#define OCP_FILTER                        COMP_Mode_HighSpeed
#define OVP_FILTER                        COMP_Mode_HighSpeed

/* Debug Setting */
#define DAC_FUNCTIONALITY                DISABLE
#define DEBUG_DAC_CH1                    DISABLE
#define DEBUG_DAC_CH2                    DISABLE
#define DEFAULT_DAC_CHANNEL_1            MC_PROTOCOL_REG_I_A
#define DEFAULT_DAC_CHANNEL_2            MC_PROTOCOL_REG_I_B
#define DEFAULT_DAC_MOTOR                0

#define DAC_TIMER_SELECTION          	 TIM3
#define DAC_TIMER_REMAPPING          	 NO_REMAP
#define DAC_TIMER_CH1_GPIO_PORT          GPIOA
#define DAC_TIMER_CH1_GPIO_PIN           GPIO_Pin_0
#define DAC_TIMER_CH2_GPIO_PORT          GPIOA
#define DAC_TIMER_CH2_GPIO_PIN           GPIO_Pin_0

#define SW_OV_CURRENT_PROT_ENABLING      DISABLE /*!< Over-current detection 
                                                         enabling */

#define START_STOP_GPIO_PORT             GPIOA
#define START_STOP_GPIO_PIN              GPIO_Pin_0
#define START_STOP_POLARITY              DIN_ACTIVE_LOW

#endif /*__CONTROL_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
