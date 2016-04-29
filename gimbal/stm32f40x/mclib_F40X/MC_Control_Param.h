/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Control_Param.h
* Author             : FOCSDK development tool ver. 1.0.0 by IMS Systems Lab
* Creation date      : Wed Sep 23 16:41:26 2015
* Description        : This file gathers parameters related to:
*                      power devices, speed regulation frequency, PID controllers
*                      setpoints and constants, start-up ramp, lowest values for
*                      speed reading validation.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CONTROL_PARAM_H
#define __MC_CONTROL_PARAM_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*********************** POWER DEVICES PARAMETERS ******************************/

/****	Power devices switching frequency  ****/
#define PWM_FREQ ((rt_uint16_t) 16000) // in Hz  (N.b.: pattern type is center aligned)

/****    Deadtime Value   ****/
#define DEADTIME_NS	((rt_uint16_t) 1000)  //in nsec; range is [0...3500]
                                                                    
/****      Uncomment the Max modulation index     ****/ 
/**** corresponding to the selected PWM frequency ****/
#define MAX_MODULATION_89_PER_CENT
//#define MAX_MODULATION_96_PER_CENT
/*********************** CURRENT REGULATION PARAMETERS ************************/

/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (3)  	// (N.b): Internal current loop is performed every
						//(REP_RATE + 1)/(2*PWM_FREQ) seconds.
						// REP_RATE has to be an odd number in case of three-shunt
						// current reading; this limitation doesn't apply to ICS

//Not to be modified
#define SAMPLING_FREQ   ((rt_uint16_t)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

/********************** POWER BOARD PROTECTIONS THRESHOLDS ********************/

#define NTC_THRESHOLD_C				60		//°C on heatsink of MB459 board
#define NTC_HYSTERIS_C				5		// Temperature hysteresis (°C)

#define OVERVOLTAGE_THRESHOLD_V		100		//Volt on DC Bus of MB459 board
#define UNDERVOLTAGE_THRESHOLD_V	7		//Volt on DC Bus of MB459 board

#define BUS_ADC_CONV_RATIO			0.006	//DC bus voltage partitioning ratio

/*********************** SPEED LOOP SAMPLING TIME *****************************/
//Not to be modified
#define PID_SPEED_SAMPLING_500us      0     // min 500usec
#define PID_SPEED_SAMPLING_1ms        1
#define PID_SPEED_SAMPLING_2ms        3     // (3+1)*500usec = 2msec
#define PID_SPEED_SAMPLING_5ms        9		// (9+1)*500usec = 5msec		
#define PID_SPEED_SAMPLING_10ms       19	// (19+1)*500usec = 10msec
#define PID_SPEED_SAMPLING_20ms       39	// (39+1)*500usec = 20msec
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms

//User should make his choice here below
#define PID_SPEED_SAMPLING_TIME   (rt_uint8_t)(1)
//#define SPEED_SAMPLING_TIME   	(rt_uint8_t)(1)
#define PID_POSITION_SAMPLING_TIME   (rt_uint8_t)(3) //2ms



/* Define here below the period of the square waved reference torque generated
 when FLUX_TORQUE_PIDs_TUNING is uncommented in STM32F10x_MCconf.h          */
#define SQUARE_WAVE_PERIOD   (rt_uint16_t)2000 //in msec

/*******           Ki, Kp, Kd COEFFICIENT CALCULATION       ********************/
/*******           		Speed control operation		     ***********************
		

              /|                /
               |               /
  	       |	      /
               |             /
               |   _________/  
               |  /
               | /
	       |/_________________________ 
	   Fmin   F_1      F_2  Fmax      
				
		                                                                

We assume a linear variation of Ki, Kp, Kd coefficients following
the motor speed. 2 intermediate frequencies ar set (see definition here after)
and 3 terms (Ki,Kp,Kd) associated with Fmin, F_1, F_2, Fmax 
(total: 4+4+4 terms); following linear coefficients are used to compute each term.

Example: 

Fmin = 500  <->	50 Hz 	(reminder -> mechanical frequency with 0.1 Hz resolution!)
Ki_min = 20	Kp_min = 40       Kd_min = 500 

F_1 = 2000 <->	200 Hz 	
Ki_1 = 80	Kp_1 = 1000        Kd_1 = 260 

then:
alpha_Ki_1 = (Ki_1-Ki_Fmin)/(F_1-Fmin) = 60/1500 = 0.04
alpha_Kp_1 = (Kp_1-Kp_Fmin)/(F_1-Fmin) = 960/1500 = 0.64
alpha_Kd_1 = (Kd_1-Kd_Fmin)/(F_1-Fmin) = -240/1500 = -0.16

** Result **
From Freq_Min to F_1, Ki, Kp, Kd will then obey to:
Ki = Ki_Fmin + alpha_Ki_1*(Freq_motor-Freq_Min)
Kp = Kp_Fmin + alpha_Kp_1*(Freq_motor-Freq_Min)
Kd = Kd_Fmin + alpha_Kd_1*(Freq_motor-Freq_Min)

		                                                                
*********************************************************************************/
//Settings for min frequency
#define Freq_Min         (rt_uint16_t)50 // 1 Hz mechanical
#define Ki_Fmin          (rt_uint16_t)8000 // Frequency min coefficient settings
#define Kp_Fmin          (rt_uint16_t)800
#define Kd_Fmin          (rt_uint16_t)0

//Settings for intermediate frequency 1
#define F_1              (rt_uint16_t)200 // 5 Hz mechanical
#define Ki_F_1           (rt_uint16_t)2000 // Intermediate frequency 1 coefficient settings
#define Kp_F_1           (rt_uint16_t)1000
#define Kd_F_1           (rt_uint16_t)0

//Settings for intermediate frequency 2
#define F_2              (rt_uint16_t)200 // 20 Hz mechanical
#define Ki_F_2           (rt_uint16_t)1000 // Intermediate frequency 2 coefficient settings
#define Kp_F_2           (rt_uint16_t)750
#define Kd_F_2           (rt_uint16_t)0

//Settings for max frequency
#define Freq_Max         (rt_uint16_t)500 // 50 Hz mechanical
#define Ki_Fmax          (rt_uint16_t)500 // Frequency max coefficient settings
#define Kp_Fmax          (rt_uint16_t)500
#define Kd_Fmax          (rt_uint16_t)0
                                                                             
/********************************************************************************/      
/* Do not modify */
/* linear coefficients */                                                                             
#define alpha_Ki_1		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Ki_F_1-(rt_int16_t)Ki_Fmin)*1024) / (rt_int32_t)(F_1-Freq_Min) )
#define alpha_Kp_1		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kp_F_1-(rt_int16_t)Kp_Fmin)*1024) / (rt_int32_t)(F_1-Freq_Min) )
#define alpha_Kd_1		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kd_F_1-(rt_int16_t)Kd_Fmin)*1024) / (rt_int32_t)(F_1-Freq_Min) )

#define alpha_Ki_2		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Ki_F_2-(rt_int16_t)Ki_F_1)*1024) / (rt_int32_t)(F_2-F_1) )
#define alpha_Kp_2		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kp_F_2-(rt_int16_t)Kp_F_1)*1024) / (rt_int32_t)(F_2-F_1) )
#define alpha_Kd_2		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kd_F_2-(rt_int16_t)Kd_F_1)*1024) / (rt_int32_t)(F_2-F_1) )

#define alpha_Ki_3		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Ki_Fmax-(rt_int16_t)Ki_F_2)*1024) / (rt_int32_t)(Freq_Max-F_2) )
#define alpha_Kp_3		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kp_Fmax-(rt_int16_t)Kp_F_2)*1024) / (rt_int32_t)(Freq_Max-F_2) )
#define alpha_Kd_3		(rt_int32_t)( ((rt_int32_t)((rt_int16_t)Kd_Fmax-(rt_int16_t)Kd_F_2)*1024) / (rt_int32_t)(Freq_Max-F_2) )


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __MC_CONTROL_PARAM_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
