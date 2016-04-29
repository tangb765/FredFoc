
#include "app_control.h"
#include "app_config.h"
#include "Attitude.h"
#include "app_protocol.h"


rt_int16_t eulerDesired = 0;


rt_int16_t eulerspeedDesired = 0;

extern rt_int32_t eulerAngles_pos[3];
extern rt_int32_t eulerAngles_spd[3];

void control_thread_entry(void* parameter)
{
	eulerActual = 0;
	eulerDesired = 0;
	
	while(1)
	{
		if (rt_sem_take(&att_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
		
		eulerActual      = eulerAngles_pos[1];
		eulerspeedActual = eulerAngles_spd[1];
		
	}
}
