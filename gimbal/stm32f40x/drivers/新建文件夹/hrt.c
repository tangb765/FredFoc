#include "hrt.h"
#include "stm32f4xx.h"
uint64_t Hrt_Time = 0;

void hrt_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = HRT_TIMER_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	

	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(HRT_TIMER_CLK, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 50000;  
	TIM_TimeBaseStructure.TIM_Prescaler = 41;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(HRT_TIMER, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(HRT_TIMER,TIM_FLAG_Update | TIM_FLAG_CC1);
	
	TIM_ITConfig(HRT_TIMER,TIM_IT_Update|TIM_IT_CC1,ENABLE);
	
	TIM_Cmd(HRT_TIMER, ENABLE);
}
hrt_abstime hrt_absolute_time(void)
{
    return Hrt_Time + HRT_TIMER->CNT;
}

hrt_abstime hrt_absolute_ms(void)
{
    return (Hrt_Time + HRT_TIMER->CNT)/2000;
}

hrt_abstime hrt_elapsed_time(hrt_abstime time)
{
	return Hrt_Time + HRT_TIMER->CNT - time;
}

#if   USE_HRT_TIMER == 4

void TIM4_IRQHandler(void)	
{
	if (TIM_GetITStatus(HRT_TIMER, TIM_IT_Update) == SET)
	{
		TIM_ClearFlag(HRT_TIMER,TIM_FLAG_Update);
		Hrt_Time += 50000;
	}
	else if (TIM_GetITStatus(HRT_TIMER, TIM_IT_CC1) == SET)
	{
		TIM_ClearFlag(HRT_TIMER,TIM_FLAG_CC1);
	}
}

#endif

#if   USE_HRT_TIMER == 5

void TIM5_IRQHandler(void)	
{
	if (TIM_GetITStatus(HRT_TIMER, TIM_IT_Update) == SET)
	{
		TIM_ClearFlag(HRT_TIMER,TIM_FLAG_Update);
		Hrt_Time += 50000;
	}
	else if (TIM_GetITStatus(HRT_TIMER, TIM_IT_CC1) == SET)
	{
		TIM_ClearFlag(HRT_TIMER,TIM_FLAG_CC1);
	}
}

#endif
