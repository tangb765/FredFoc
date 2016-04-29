#ifndef _BROADCAST_TEST_H_
#define _BROADCAST_TEST_H_
#include <rtthread.h>

struct MSG
{
	rt_uint8_t head;
	rt_size_t  size;
	rt_uint8_t msg_body[64];
};

int app_broadcast(void);
#endif
