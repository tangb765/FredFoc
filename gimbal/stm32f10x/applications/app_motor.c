
#include "mclib_enter_API.h"

void motor_thread_entry(void* parameter)
{
	Mclib_API_thread_entry(parameter);
}


void app_motor_init(void)
{
	MClib_application_init();
	
	return;
}
