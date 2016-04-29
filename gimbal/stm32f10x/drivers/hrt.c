#include "hrt.h"
#include "stm32f10x.h"
uint64_t Hrt_Time = 0;

hrt_abstime hrt_absolute_time(void)
{
    return Hrt_Time + SysTick->VAL;
}

hrt_abstime hrt_absolute_us(void)
{
    return (Hrt_Time + SysTick->VAL)/HRT_US_COUNT;
}

hrt_abstime hrt_absolute_ms(void)
{
    return (Hrt_Time + SysTick->VAL)/HRT_MS_COUNT;
}

hrt_abstime hrt_elapsed_ustime(hrt_abstime time)
{
	return Hrt_Time + SysTick->VAL - time*HRT_US_COUNT;
}

hrt_abstime hrt_elapsed_mstime(hrt_abstime time)
{
	return Hrt_Time + SysTick->VAL - time*HRT_MS_COUNT;
}

void delay_ms(hrt_abstime ms)
{
	hrt_abstime current_ms;
	current_ms = hrt_absolute_ms();
	
	while(current_ms+ms >= hrt_absolute_ms());
}
