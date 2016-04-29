
#include "app_control.h"
#include "Attitude.h"
#include "AttitudeEKF.h"
#include "MPU6000.h"
#include "hrt.h"
#include "app_config.h"
#include "app_protocol.h"

rt_int16_t eulerActual = 0;
rt_int16_t eulerDesired = 0;

rt_int16_t eulerspeedActual = 0;
rt_int16_t eulerspeedDesired = 0;
	
extern int32_t eulerAngles_pos[3];
extern int32_t eulerAngles_spd[3];

void control_thread_entry(void* parameter)
{
	eulerActual = 0;
	eulerDesired = 0;
	
	while(1)
	{
		if (rt_sem_take(&att_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
		
		eulerActual = eulerAngles_pos[1];
		eulerspeedActual = eulerAngles_spd[1];
		
	}
}
