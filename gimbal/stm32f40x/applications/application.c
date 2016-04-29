/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif


#include "TLE5012.h"
#include "app_broadcast.h"
#include "test.h"
//#include "app_control.h"
#include "app_motor.h"
#ifdef FeiYu_Board_A	
#include "app_mpu6500.h"
#include "Attitude.h"
#endif

void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif	
		
		app_broadcast();
	  TLE5012_init();
	  app_motor_init();

#ifdef FeiYu_Board_A	
		app_mpu6500_init();
		app_attitude_init();
#endif
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
		
		
    /* init motor thread */
    tid = rt_thread_create("motor",
                            motor_thread_entry,
                            RT_NULL,
                            2048,
                            RT_THREAD_PRIORITY_MAX/3+0,
                            5);
    if (tid  != RT_NULL)
    {
        rt_thread_startup(tid);
    }

#ifdef FeiYu_Board_A
    tid = rt_thread_create("mpu6500",
                            mpu6500_thread_entry,
                            RT_NULL,
                            1024,
                            RT_THREAD_PRIORITY_MAX/3+1,
                            5);
    if (tid  != RT_NULL)
    {
        rt_thread_startup(tid);
    }

		/* init attitude thread */
		tid = rt_thread_create("attitude",
														attitude_thread_entry,
														RT_NULL,
														4096,
														RT_THREAD_PRIORITY_MAX/3+1,
														5);	

		if (tid  != RT_NULL)
		{
				rt_thread_startup(tid);
		}
		
#endif
//		
//		/* init control thread */
//		tid = rt_thread_create("control",
//														control_thread_entry,
//														RT_NULL,
//														4096,
//														RT_THREAD_PRIORITY_MAX/3+1,
//														5);	

//		if (tid  != RT_NULL)
//		{
//				rt_thread_startup(tid);
//		}
		
		
		/* init test thread */
		tid = rt_thread_create("test",
														test_thread_entry,
														RT_NULL,
														1024,
														RT_THREAD_PRIORITY_MAX/3+5,
														5);	

		if (tid  != RT_NULL)
		{
				rt_thread_startup(tid);
		}
			
    return 0;
}

/*@}*/
