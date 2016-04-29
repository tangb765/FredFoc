
#include "app_control.h"
#include "MC_Globals.h"

rt_int16_t eulerActual;
rt_int16_t eulerDesired;



static void control_thread_entry(void* parameter)
{
	eulerActual = 0;
	eulerDesired = 0;
	
	while(1)
	{
		rt_thread_delay( RT_TICK_PER_SECOND / 500 ); 

	}
}


int app_control_init(void)
{
	rt_thread_t tid;

	/* init led thread */
	tid = rt_thread_create("control",
													control_thread_entry,
													RT_NULL,
													512,
													20,
													5);	

	if (tid  != RT_NULL)
	{
			rt_thread_startup(tid);
	}
	
	return 0;
}
