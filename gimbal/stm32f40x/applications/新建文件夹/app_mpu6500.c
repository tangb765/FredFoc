#include "app_mpu6500.h"
#include "MPU6000.h"
#include "stm32f20x_40x_spi.h"
#include <drivers/spi.h>
#include "hrt.h"
#include "finsh.h"

//ORB_DEFINE(accelerometer_raw,accelerometer_raw_s);
//ORB_DEFINE(gyroscopes_raw,gyroscopes_raw_s);

typedef struct{

	uint64_t timestamp;	/**< in microseconds since system start  */

	rt_int16_t data[3];
	float scale;
	bool   valid; 

}accelerometer_raw_s;

typedef struct{

	uint64_t timestamp;	/**< in microseconds since system start */

	rt_int16_t data[3];
	float scale;
	bool   valid; 
}gyroscopes_raw_s;

bool run_6500 = true;
bool mpu6500_running = false;

void mpu6500(char* cmd)
{
	rt_thread_t tid = RT_NULL;
	if(strcmp(cmd,"start") == 0){
		
		if(mpu6500_running == true){
			rt_kprintf("mpu6500 is already started!\r\n");
		}else{
			run_6500 = true;
			tid = rt_thread_create("mpu6500",
				app_mpu6500_main, RT_NULL,
				1024, 5, 10);
			if (tid != RT_NULL)
				rt_thread_startup(tid);
		}
	}else if(strcmp(cmd,"stop") == 0){
		if(mpu6500_running == false){
			rt_kprintf("mpu6500 is already stopped!\r\n");
		}else{
			run_6500 = false;
		}
	}
	else{
		rt_kprintf(" %s command not found\r\n",cmd);
		rt_kprintf("usage:mpu6500 {start|stop}\r\n");
	}
}


void app_mpu6500_main(void* parameter)
{
	rt_uint32_t i;
	rt_int16_t acc_raw[3];
	rt_int16_t gyro_raw[3];
	
	accelerometer_raw_s acc;
	gyroscopes_raw_s    gyro;
	mpu6500_running = true;
	rt_kprintf("mpu6500 start!\r\n");
	while(run_6500)
	{
		if(!(MPU6500_Init() == RT_EOK)){
			rt_kprintf("ERROR:MPU6500 selftest failed !\r\n");
		}else{
			rt_kprintf("MPU6500 init success.\n");
			break;
		}
		rt_thread_delay(RT_TICK_PER_SECOND * 5);
	}
	
	acc.scale = MPU6500_GetFullScaleAccelGPL() * GRAVITY;
	gyro.scale = MPU6500_GetFullScaleGyroDPL() * DEGTORAD;
	
	/* main loop*/
	while(run_6500){
		
		MPU6500_GetMotion6(&acc_raw[0],
		                   &acc_raw[1],
		                   &acc_raw[2],
		                   &gyro_raw[0],
		                   &gyro_raw[1],
		                   &gyro_raw[2]);
		
		/* we can do rotation here if we need*/
		
		/*rotate to NED coordinates*/
		acc_raw[2] = (acc_raw[2] == -32768)?32767:-acc_raw[2];
		acc_raw[0] = (acc_raw[0] == -32768)?32767:-acc_raw[0];

		gyro_raw[0] = (gyro_raw[0] == -32768)?32767:-gyro_raw[0];
		gyro_raw[2] = (gyro_raw[2] == -32768)?32767:-gyro_raw[2];
		
		for(i = 0;i < 3;i++)
		    acc.data[i]  = acc_raw[i];
		acc.timestamp = hrt_absolute_time();
		acc.valid = true;
		
		for(i = 0;i < 3;i++)
		    gyro.data[i] = gyro_raw[i];
		gyro.timestamp = hrt_absolute_time();
		
		gyro.valid = true;
		
//		orb_publish(ORB_ID(accelerometer_raw),&acc);
//		orb_publish(ORB_ID(gyroscopes_raw),&gyro);
		rt_thread_delay(RT_TICK_PER_SECOND/500);
	}
	mpu6500_running = false;
	rt_kprintf("MPU6500 stopped!\r\n");
}

//FINSH_FUNCTION_EXPORT(mpu6500, mpu6500 control interface);
