/******************************************************************************/
/*                                                                            */
/*                               MC_Globals.c                                 */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include "board.h"
#include "MC_const.h"
#include "MC_type.h"
#include "MC_Globals.h"
#include "stm32f10x_encoder.h"
#include "app_config.h"
/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/



/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
/* Electrical, magnetic and mechanical variables*/

Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                            transformations of Ia & Ib */

Curr_Components Stat_Curr_q_d;              /*Iq & Id, Parke's transformations of 
                                            Ialpha & Ibeta, */

Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

Volt_Components Stat_Volt_q_d;              /*Vq & Vd, voltages on a reference
                                            frame synchronous with the rotor flux*/

Volt_Components Stat_Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations
                                             of Vq & Vd*/

/*Variable of convenience*/

volatile rt_uint32_t wGlobal_Flags =  SPEED_CONTROL;//TORQUE_CONTROL;
//volatile rt_uint32_t wGlobal_Flags = FIRST_START | SPEED_CONTROL;
volatile SystStatus_t State = IDLE;//IDLE;INIT

volatile rt_int16_t hFlux_Reference = 0;

volatile rt_int16_t hTorque_Reference = 0;

volatile rt_int16_t hSpeed_Reference = 0;

volatile rt_int16_t hPosition_reference =0;

volatile MCLIB_Measure_variale_Struct_t MCLIB_Mesure_Structure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
rt_int16_t Get_Mec_Angle_Value(void)
{
	return (rt_int16_t)(MCLIB_Mesure_Structure.Mec_Angle - configs.level_mec_angle);
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
