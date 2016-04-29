/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2013-07-12     aozima       update for auto initial.
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f10x.h"
#include "board.h"
#include "usart.h"
#include "gpio.h"

#include "hrt.h"
#include "app_config.h"

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */


/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
rt_uint32_t  cnts;
static RCC_ClocksTypeDef  rcc_clocks;
void  SysTick_Configuration(void)
{
    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config(cnts);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

		Hrt_Time += cnts;
	
    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}



/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();
	
	 Init_Config();

    /* Configure the SysTick */
    SysTick_Configuration();

    stm32_hw_usart_init();
//    stm32_hw_pin_init();  
	
#ifdef FeiYu_Board_Ax
		stm32_hw_spi_init();
#endif
	
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
		
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

/*@}*/
