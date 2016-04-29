#ifndef __HRT_H_
#define __HRT_H_

#include "stdint.h"

#define HRT_MS_COUNT 9000
#define HRT_US_COUNT 9

extern uint64_t Hrt_Time;

typedef uint64_t	hrt_abstime;

hrt_abstime hrt_absolute_time(void);
hrt_abstime hrt_absolute_us(void);
hrt_abstime hrt_absolute_ms(void);
hrt_abstime hrt_elapsed_ustime(hrt_abstime time);
hrt_abstime hrt_elapsed_mstime(hrt_abstime time);
void delay_ms(hrt_abstime ms);

#endif
