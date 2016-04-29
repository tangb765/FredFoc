#include <rtthread.h>
#include "board.h"
#include "app_protocol.h"
#include "TLE5012.h"


#ifdef FeiYu_Board_Ax
#include "Attitude.h"
#endif

#ifdef FeiYu_Board_A
#define BOARD  'A'
#endif

#ifdef FeiYu_Board_B
#define BOARD  'B'
#endif

#ifdef FeiYu_Board_C
#define BOARD  'C'
#endif

static rt_uint8_t board = BOARD;


#include "MC_Globals.h"
extern rt_uint8_t speed_x;
void test_thread_entry(void *parameter)
{
	rt_size_t count = 0;

	while (1)
	{
		rt_thread_delay( RT_TICK_PER_SECOND / 500 ); 
		
		count++;
		
		if(count>36000) count = 0;	
		
		switch (output_flag)
		{
			case CLOSE:
				break;
			
			case TLE5012:
				rt_kprintf("%c,tle5012,%d\r\n", board,TLE5012_Postion());
				break;

#ifdef FeiYu_Board_Ax
			case MPU6500:
				rt_kprintf("%c,mpu6500,%d,%d,%d,%d,%d,%d\r\n", board,imu.gyro_i16.x,imu.gyro_i16.y,imu.gyro_i16.z,imu.acc_i16.x,imu.acc_i16.y,imu.acc_i16.z);
				break;
#endif
			case MOTOR:
				rt_kprintf("%c:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", board,motor_state,hPosition_reference,Get_Mec_Angle_Value(),hSpeed_Reference*speed_x,
										MCLIB_Mesure_Structure.Mec_Speed,hTorque_Reference,Stat_Curr_q_d.qI_Component1,hFlux_Reference,Stat_Curr_q_d.qI_Component2,eulerActual);					
				break;
			
			case IMU:
				break;
			
			default:
				break;
		}
	}
}

