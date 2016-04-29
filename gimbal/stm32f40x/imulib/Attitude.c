
#include "stdio.h"
#include "Attitude.h"
#include "AttitudeEKF.h"
#include "MPU6000.h"
#include "board.h"
#include "hrt.h"
#include "app_config.h"
#include "app_protocol.h"
#include "sensfusion6.h"
#include "MC_Globals.h"

rt_uint8_t approx_prediction = 0;
rt_uint8_t use_inertia_matrix = 0;
const rt_uint8_t zFlag[3] = {1,1,0};
float dt = 0.002f;
float z[9] = {0.0f,0.0f,0.0f,0.0f,0.0f,9.81f,0.2f,-0.2f,0.2f};
float q_rotSpeed = 0.0001f;
float q_rotAcc = 0.08f;
float q_acc = 0.009f;
float q_mag = 0.005f;
float r_gyro = 0.0008f;
float r_accel = 10000.0f;
float r_mag = 100.0f;
const float J[9] = {0.0018,0,0,0,0.0018,0,0,0,0.0037};
//  float xa_apo[12];
//	float Pa_apo[144];
float xa_apo[9];
float Pa_apo[81];	
float Rot_matrix[9] = {1.f,  0,  0,
											 0,  1.f,  0,
											 0,  0,  1.f
										  };
float eulerAngles[3] = {0.0f, 0.0f, 0.0f};
float debugOutput[4] = { 0.0f };
rt_uint8_t buf_tmp[64];
uint64_t times1 = 0,times2 = 0;
static uint64_t last_timestamp = 0;

rt_int16_t eulerAngles_pos[3];
rt_int16_t eulerAngles_spd[3];

void attitude_thread_entry(void* parameter)
{

	while(1)
	{
		if (rt_sem_take(&att_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
		
		if(first_times<50)
		{
			if(first_times == 0) State = INIT;//IDLE;//INIT;
			output_flag = IMU;
			first_times++;
			last_timestamp = imu.timestamp;
			continue;
		}

		if (imu.timestamp - last_timestamp > HRT_MS_COUNT*8) 
		{
			if(first_times == 50) 
			{
				State = STOP;
				rt_kprintf("%c,att timeout.\r\n",CONSOLE_O-1+'A');
				output_flag = CLOSE;
			}
			
			first_times = 51;
			
			continue;
		}
		else
		{
			last_timestamp = imu.timestamp;
		}
	
		z[0] = imu.gyro_f32.x*0.017453292f;
		z[1] = imu.gyro_f32.y*0.017453292f;
		z[2] = imu.gyro_f32.z*0.017453292f;
		
		z[3] = imu.acc_f32.x*9.81f;
		z[4] = imu.acc_f32.y*9.81f;
		z[5] = imu.acc_f32.z*9.81f;
		
		if((configs.imu.iscalibed&0x0100) && (configs.imu.iscalibed&0x0001))
		{
//			times1 = hrt_absolute_time();
			
			AttitudeEKF(approx_prediction,use_inertia_matrix,zFlag,dt,z,q_rotSpeed,q_rotAcc,q_acc,q_mag,r_gyro,r_accel,r_mag,J,xa_apo,Pa_apo,Rot_matrix,eulerAngles,debugOutput);
//			sensfusion6UpdateQ(imu.gyro_f32.x, imu.gyro_f32.y, imu.gyro_f32.z, imu.acc_f32.x, imu.acc_f32.y, imu.acc_f32.z, dt);
//			sensfusion6GetEulerRPY(&eulerAngles[0], &eulerAngles[1], &eulerAngles[2]);
			
//			times2 = hrt_absolute_time();
			
			eulerAngles_pos[0] = eulerAngles[0] * 182.044444f;
			eulerAngles_pos[1] = -eulerAngles[1] * 182.044444f;
			eulerAngles_pos[2] = eulerAngles[2] * 182.044444f;	

			eulerAngles_spd[0] = (int32_t)imu.gyro_f32.x;
			eulerAngles_spd[1] = (int32_t)(-imu.gyro_f32.y);
			eulerAngles_spd[2] = (int32_t)imu.gyro_f32.z;

			eulerActual = -eulerAngles_pos[0];
			eulerspeedActual = -eulerAngles_spd[0];
			
			if (output_flag == IMU)
			{
				rt_kprintf("#E,%d,%d,%d,%d,%d,%d$\r\n",-eulerAngles_pos[0],eulerAngles_pos[1],-eulerAngles_pos[2],-eulerAngles_spd[0],eulerAngles_spd[1],-eulerAngles_spd[2]);
			}
		}
	}
}


int app_attitude_init(void)
{
	AttitudeEKF_initialize();
	sensfusion6Init();
	return 0;
}

