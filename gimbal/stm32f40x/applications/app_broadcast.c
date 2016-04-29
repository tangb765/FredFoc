#include "board.h"
#include "app_broadcast.h"
#include "app_protocol.h"

#define BUFSIZE   64*4
#define BUFSIZE1  64


struct rt_app_uart_rx_fifo
{
	/* software fifo */
	rt_uint8_t buffer[BUFSIZE1];

	rt_uint16_t put_index, get_index;
};


//struct rt_app_uart_rx_fifo app_uart1_rx_fifo;

rt_size_t uart1_rx_size = 0;
rt_size_t uart3_rx_size = 0;
struct rt_semaphore uart1_rx_sem;
struct rt_semaphore uart3_rx_sem;
rt_device_t device, device3;
static struct rt_broadcast_server s_broadcast_server;
static struct rt_broadcast_server s_broadcast_server2;

static struct rt_broadcast_client s_client_1; 
static rt_uint8_t s_listener1_mq_buffer[BUFSIZE];
static struct rt_thread s_client_entry1;
static rt_int8_t s_client_entry1_stack[512];


static struct rt_broadcast_client s_client_2; 
static rt_uint8_t s_listener2_mq_buffer[BUFSIZE];
static struct rt_thread s_client_entry2;
static rt_int8_t s_client_entry2_stack[512];


static struct rt_broadcast_client s_client_3; 
static rt_uint8_t s_listener3_mq_buffer[BUFSIZE];
static struct rt_thread s_client_entry3;
static rt_int8_t s_client_entry3_stack[512];


//static struct rt_broadcast_client s_client_4; 
//static rt_uint8_t s_listener4_mq_buffer[BUFSIZE];
//static struct rt_thread s_client_entry4;
//static rt_int8_t s_client_entry4_stack[BUFSIZE];


//static struct rt_broadcast_client s_client_5; 
//static rt_uint8_t s_listener5_mq_buffer[BUFSIZE];
//static struct rt_thread s_client_entry5;
//static rt_int8_t s_client_entry5_stack[BUFSIZE];


//static struct rt_broadcast_client s_client_6; 
//static rt_uint8_t s_listener6_mq_buffer[BUFSIZE];
//static struct rt_thread s_client_entry6;
//static rt_int8_t s_client_entry6_stack[BUFSIZE];


//static struct rt_broadcast_client s_client_7; 
//static rt_uint8_t s_listener7_mq_buffer[BUFSIZE];
//static struct rt_thread s_client_entry7;
//static rt_int8_t s_client_entry7_stack[BUFSIZE];

//debug info passthrough
static void broadcast_thread_entry_rev1(void* parameter)
{
	struct MSG msg;
	rt_memset(&msg,0,sizeof(struct MSG));
	
	while(1)
	{
		if(RT_EOK ==rt_broadcast_recv(&s_client_1,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
		{
			if (msg.head==1)
			{
				rt_device_write(device, 0, msg.msg_body, msg.size);
				rt_memset(&msg,0,sizeof(struct MSG));
			}
		}
	}
}



//imu control cmd:#.......$
static void broadcast_thread_entry_rev2(void* parameter)
{
	struct MSG msg;
	rt_memset(&msg,0,sizeof(struct MSG));
	
	while(1)
	{
		if(RT_EOK ==rt_broadcast_recv(&s_client_2,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
		{
			if (msg.head==2)
			{
//				#ifndef FeiYu_Board_C	
				rt_device_write(device, 0, msg.msg_body, msg.size);
//				#endif
				imu_protocol(&msg.msg_body[0],msg.size);
				rt_memset(&msg,0,sizeof(struct MSG));
			}
		}
	}
}

//extern control cmd:#.......@
static void broadcast_thread_entry_rev3(void* parameter)
{
	struct MSG msg;
	rt_memset(&msg,0,sizeof(struct MSG));
	
	while(1)
	{
		
		if(RT_EOK ==rt_broadcast_recv(&s_client_3,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
		{
			if (msg.head==3)
			{
				#ifdef RT_USING_UART3	
				rt_device_write(device3, 0, msg.msg_body, msg.size);
				#endif
				extern_cmd_protocol(&msg.msg_body[0],msg.size);
				rt_memset(&msg,0,sizeof(struct MSG));
			}
			
			if (msg.head==4)
			{
				msg.msg_body[msg.size-1] = '\r';
//				finsh_protocol(&msg.msg_body[1],msg.size-1);
				
				rt_memset(&msg,0,sizeof(struct MSG));
			}
			
			if (msg.head==5)	
			{
				#ifdef RT_USING_UART3	
				rt_device_write(device3, 0, msg.msg_body, msg.size);
				#endif
				rt_memset(&msg,0,sizeof(struct MSG));
			}
		}
	}
}

////finsh cmd:#.......*
//static void broadcast_thread_entry_rev4(void* parameter)
//{
//	struct MSG msg;
//	
//	rt_memset(&msg,0,sizeof(struct MSG));
//	
//	while(1)
//	{
//		if(RT_EOK ==rt_broadcast_recv(&s_client_4,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
//		{
//			if (msg.head==4)
//			{
//				msg.msg_body[msg.size-1] = '\r';
//				finsh_protocol(&msg.msg_body[1],msg.size-1);
//				
//				rt_memset(&msg,0,sizeof(struct MSG));
//			}
//		}
//	}
//}

////passthtough finsh cmd:#.......*
//static void broadcast_thread_entry_rev5(void* parameter)
//{
//	struct MSG msg;
//	rt_memset(&msg,0,sizeof(struct MSG));
//	
//	while(1)
//	{
//		
//		if(RT_EOK ==rt_broadcast_recv(&s_client_5,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
//		{
//			if (msg.head==5)	
//			{
//				rt_device_write(device3, 0, msg.msg_body, msg.size);
//				rt_memset(&msg,0,sizeof(struct MSG));
//			}
//		}
//	}
//}

//static void broadcast_thread_entry_rev6(void* parameter)
//{
//	struct MSG msg;
//	rt_memset(&msg,0,sizeof(struct MSG));
//	
//	while(1)
//	{
//		
//		if(RT_EOK ==rt_broadcast_recv(&s_client_6,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
//		{
//			if (msg.head==6)
//			{
//				rt_device_write(device, 0, msg.msg_body, msg.size);
//				rt_memset(&msg,0,sizeof(struct MSG));
//			}
//		}
//	}
//}

//static void broadcast_thread_entry_rev7(void* parameter)
//{
//	struct MSG msg;
//	rt_memset(&msg,0,sizeof(struct MSG));
//	
//	while(1)
//	{
//		
//		if(RT_EOK ==rt_broadcast_recv(&s_client_7,&msg,sizeof(struct MSG),RT_WAITING_FOREVER))
//		{
//			if (msg.head==7)
//			{
////				rt_kprintf("<-client7 recv msg:%02x  %s\n",msg.head,msg.msg_body);
//				rt_memset(&msg,0,sizeof(struct MSG));
//			}
//		}
//	}
//}

//check sum:BCC algorithm
rt_uint8_t Checksum(rt_uint8_t *p)
{
    rt_uint8_t checksum=0;
    rt_uint8_t i=0;
		
		while(p[i] != '*')
    {
        checksum ^=p[i];
        i++;
    }
		
    return checksum;
}

static struct rt_thread s_send_entry1;
static rt_int8_t s_send1_entry_stack[BUFSIZE];

//static struct rt_thread s_send_entry2;
//static rt_int8_t s_send2_entry_stack[BUFSIZE];

static struct rt_thread s_send_entry3;
static rt_int8_t s_send3_entry_stack[BUFSIZE];

static rt_uint8_t send1_buf[BUFSIZE1];
static rt_uint8_t send3_buf[BUFSIZE1];

static void broadcast_thread_entry_send1(void *parameter)
{
	struct MSG msg;
	rt_err_t  err;
	rt_size_t buf_tmp_start=0;
	rt_size_t buf_tmp_end=0;
	rt_size_t buf_pos_i = 0;
	rt_size_t buf_pos_f = 0;
	rt_size_t buf_pos_n = 0;
	rt_size_t i=0;
	
	while(1)
	{
		/* wait receive */
    if (rt_sem_take(&uart1_rx_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
		
		if (uart1_rx_size == 0) continue;

		rt_device_read(device, 0, send1_buf, uart1_rx_size);
		
		buf_pos_i = 0;
		buf_pos_f = 0;
		buf_pos_n = 0;
		buf_tmp_start = 0;
		buf_tmp_end = 0;
		
		while(buf_pos_i<uart1_rx_size) 
		{
			if (send1_buf[buf_pos_i] == '#')
			{	
				buf_pos_f++;
				buf_tmp_start = buf_pos_i;
				buf_pos_i++;
				continue;
			}
			
			//imu control cmd
			if(send1_buf[buf_pos_i] == '$' && buf_pos_f!=0)
			{
				rt_size_t size_tmp;
				
				for(i=0;i<4;i++)
				{
					if (buf_pos_i+1<uart1_rx_size)
					{
						if(send1_buf[buf_pos_i+1] == '\r' || send1_buf[buf_pos_i+1] == '\n')
						{
							buf_pos_i++;
						}
					}
				}

				size_tmp = buf_pos_i-buf_tmp_start+1;
				buf_pos_n++;
				buf_pos_f=0;
				buf_tmp_end = buf_pos_i+1;

				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_start],size_tmp);
				msg.head =2;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send1 msg2 failed:%d\r\n", err);
				}
				
				buf_pos_i++;
				continue;
			}
			
			//extern control cmd
			if(send1_buf[buf_pos_i] == '@' && buf_pos_f!=0)
			{
				rt_size_t size_tmp;
				
				for(i=0;i<4;i++)
				{
					if (buf_pos_i+1<uart1_rx_size)
					{
						if(send1_buf[buf_pos_i+1] == '\r' || send1_buf[buf_pos_i+1] == '\n')
						{
							buf_pos_i++;
						}
					}
				}

				size_tmp = buf_pos_i-buf_tmp_start+1;
				buf_pos_n++;
				buf_pos_f=0;
				buf_tmp_end = buf_pos_i+1;

				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_start],size_tmp);
				msg.head =3;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server2, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send1 msg3 failed:%d\r\n", err);
				}	
				
				buf_pos_i++;
				continue;
			}

			//finsh cmd
			if ((console_flag&0x0f) == CONSOLE_O)
			{
				if(send1_buf[buf_pos_i] == '*' && buf_pos_f!=0)
				{
					rt_size_t size_tmp;

					size_tmp = buf_pos_i-buf_tmp_start+1;
					buf_pos_n++;
					buf_pos_f=0;
					
					for(i=0;i<4;i++)
					{
						if (buf_pos_i+1<uart1_rx_size)
						{
							if(send1_buf[buf_pos_i+1] == '\r' || send1_buf[buf_pos_i+1] == '\n')
							{
								buf_pos_i++;
							}
						}
					}
					
					buf_tmp_end = buf_pos_i+1;

					rt_memset(&msg, 0, sizeof(struct MSG));
					rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_start],size_tmp);
					msg.head =4;
					msg.size =size_tmp;
					
					err = rt_broadcast_send(&s_broadcast_server2, &msg, sizeof(struct MSG));
					
					if(RT_EOK != err)
					{
							rt_kprintf("send1 msg4 failed:%d\r\n", err);
					}	
					
					buf_pos_i++;
					continue;
				}	
			}
			else
			{
				//passthtough finsh cmd
				if(send1_buf[buf_pos_i] == '*' && buf_pos_f!=0)
				{
					rt_size_t size_tmp;
					
					for(i=0;i<4;i++)
					{
						if (buf_pos_i+1<uart1_rx_size)
						{
							if(send1_buf[buf_pos_i+1] == '\r' || send1_buf[buf_pos_i+1] == '\n')// || send1_buf[buf_pos_i+1] == 0)
							{
								buf_pos_i++;
							}
						}
					}

					size_tmp = buf_pos_i-buf_tmp_start+1;
					buf_pos_n++;
					buf_pos_f=0;
					buf_tmp_end = buf_pos_i+1;
					

					rt_memset(&msg, 0, sizeof(struct MSG));
					rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_start],size_tmp);
					msg.head =5;
					msg.size =size_tmp;
					
					err = rt_broadcast_send(&s_broadcast_server2, &msg, sizeof(struct MSG));
					
					if(RT_EOK != err)
					{
							rt_kprintf("send1 msg5 failed:%d\r\n", err);
					}

					buf_pos_i++;
					continue;					
				}	
			}
			
			if(buf_pos_f!=0 && buf_tmp_end != buf_tmp_start)
			{
				rt_size_t size_tmp;
				size_tmp = buf_tmp_start-buf_tmp_end;
				
				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_end],size_tmp);
				msg.head =1;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send1 msg1 failed:%d\r\n", err);
				}
				buf_tmp_end = buf_tmp_start;
				
				buf_pos_i++;
				continue;
			}
			
			buf_pos_i++;
		}
		
		
		if (buf_tmp_end<uart1_rx_size)
		{
			rt_size_t size_tmp;
			size_tmp = uart1_rx_size-buf_tmp_end;
			
			rt_memset(&msg, 0, sizeof(struct MSG));
			rt_memcpy(msg.msg_body,&send1_buf[buf_tmp_end],size_tmp);
			msg.head =1;
			msg.size =size_tmp;
			
			err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
			
			if(RT_EOK != err)
			{
					rt_kprintf("send1 msg6 failed:%d\r\n", err);
			}
		}
	
		uart1_rx_size = 0;
	}
}

static void broadcast_thread_entry_send3(void *parameter)
{
	struct MSG msg;
	rt_err_t  err;
	rt_size_t buf_tmp_start=0;
	rt_size_t buf_tmp_end=0;
	rt_size_t buf_pos_i = 0;
	rt_size_t buf_pos_f = 0;
	rt_size_t buf_pos_n = 0;
	rt_size_t i=0;
	
	while(1)
	{
		/* wait receive */
    if (rt_sem_take(&uart3_rx_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
		
		if (uart3_rx_size == 0) continue;

		rt_device_read(device3, 0, send3_buf, uart3_rx_size);
		
		buf_pos_i = 0;
		buf_pos_f = 0;
		buf_pos_n = 0;
		buf_tmp_start = 0;
		buf_tmp_end = 0;
		
		while(buf_pos_i<uart3_rx_size) 
		{
			if (send3_buf[buf_pos_i] == '#')
			{	
				buf_pos_f++;
				buf_tmp_start = buf_pos_i;
				buf_pos_i++;
				continue;
			}
			
			//imu control cmd
			if(send3_buf[buf_pos_i] == '$' && buf_pos_f!=0)
			{
				rt_size_t size_tmp;
				
				for(i=0;i<4;i++)
				{
					if (buf_pos_i+1<uart3_rx_size)
					{
						if(send3_buf[buf_pos_i+1] == '\r' || send3_buf[buf_pos_i+1] == '\n')
						{
							buf_pos_i++;
						}
					}
				}

				size_tmp = buf_pos_i-buf_tmp_start+1;
				buf_pos_n++;
				buf_pos_f=0;
				buf_tmp_end = buf_pos_i+1;

				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_start],size_tmp);
				msg.head =2;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send3 msg1 failed:%d\r\n", err);
				}
				
				buf_pos_i++;
				continue;
			}
			
			//extern control cmd
			if(send3_buf[buf_pos_i] == '@' && buf_pos_f!=0)
			{
				rt_size_t size_tmp;
				
				for(i=0;i<4;i++)
				{
					if (buf_pos_i+1<uart3_rx_size)
					{
						if(send3_buf[buf_pos_i+1] == '\r' || send3_buf[buf_pos_i+1] == '\n')
						{
							buf_pos_i++;
						}
					}
				}

				size_tmp = buf_pos_i-buf_tmp_start+1;
				buf_pos_n++;
				buf_pos_f=0;
				buf_tmp_end = buf_pos_i+1;

				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_start],size_tmp);
				msg.head =3;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send3 msg2 failed:%d\r\n", err);
				}	
				
				buf_pos_i++;
				continue;
			}

			//finsh cmd
			if ((console_flag&0x0f) == CONSOLE_O)
			{
				if(send3_buf[buf_pos_i] == '*' && buf_pos_f!=0)
				{
					rt_size_t size_tmp;
					
					for(i=0;i<4;i++)
					{
						if (buf_pos_i+1<uart3_rx_size)
						{
							if(send3_buf[buf_pos_i+1] == '\r' || send3_buf[buf_pos_i+1] == '\n')
							{
								buf_pos_i++;
							}
						}
					}

					size_tmp = buf_pos_i-buf_tmp_start+1;
					buf_pos_n++;
					buf_pos_f=0;
					buf_tmp_end = buf_pos_i+1;

					rt_memset(&msg, 0, sizeof(struct MSG));
					rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_start],size_tmp);
					msg.head =4;
					msg.size =size_tmp;
					
					err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
					
					if(RT_EOK != err)
					{
							rt_kprintf("send3 msg3 failed:%d\r\n", err);
					}

					buf_pos_i++;
					continue;	
				}	
			}
			else
			{
				//passthtough finsh cmd
				if(send3_buf[buf_pos_i] == '*' && buf_pos_f!=0)
				{
					rt_size_t size_tmp;
					
					for(i=0;i<4;i++)
					{
						if (buf_pos_i+1<uart3_rx_size)
						{
							if(send3_buf[buf_pos_i+1] == '\r' || send3_buf[buf_pos_i+1] == '\n')// || send3_buf[buf_pos_i+1] == 0)
							{
								buf_pos_i++;
							}
						}
					}

					size_tmp = buf_pos_i-buf_tmp_start+1;
					buf_pos_n++;
					buf_pos_f=0;
					buf_tmp_end = buf_pos_i+1;
					

					rt_memset(&msg, 0, sizeof(struct MSG));
					rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_start],size_tmp);
					msg.head =5;
					msg.size =size_tmp;
					
					err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
					
					if(RT_EOK != err)
					{
							rt_kprintf("send3 msg4 failed:%d\r\n", err);
					}

					buf_pos_i++;
					continue;				
				}	
			}
			
			if(buf_pos_f!=0 && buf_tmp_end != buf_tmp_start)
			{
				rt_size_t size_tmp;
				size_tmp = buf_tmp_start-buf_tmp_end;
				
				rt_memset(&msg, 0, sizeof(struct MSG));
				rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_end],size_tmp);
				msg.head =1;
				msg.size =size_tmp;
				
				err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
				
				if(RT_EOK != err)
				{
						rt_kprintf("send3 msg5 failed:%d\r\n", err);
				}
				buf_tmp_end = buf_tmp_start;
				
				buf_pos_i++;
				continue;
			}
			
			
			buf_pos_i++;
		}
		
		
		if (buf_tmp_end<uart3_rx_size)
		{
			rt_size_t size_tmp;
			size_tmp = uart3_rx_size-buf_tmp_end;
			
			rt_memset(&msg, 0, sizeof(struct MSG));
			rt_memcpy(msg.msg_body,&send3_buf[buf_tmp_end],size_tmp);
			msg.head =1;
			msg.size =size_tmp;
			
			err = rt_broadcast_send(&s_broadcast_server, &msg, sizeof(struct MSG));
			
			if(RT_EOK != err)
			{
					rt_kprintf("send3 msg6 failed:%d\r\n", err);
			}
		}
	
		uart3_rx_size = 0;
	}
}


//static void broadcast_thread_entry_send2(void *parameter)
//{
//	struct MSG msg;
//	rt_uint8_t temp =0;
//	rt_err_t err;

//	while(1)
//	{
//		rt_memset(&msg,0,sizeof(struct MSG));
//		msg.head =6;//temp ++;
//		if(temp>7) temp=0;
//		strncpy(msg.msg_body,"entry_send2 msg",sizeof(msg.msg_body));
//		err =rt_broadcast_send(&s_broadcast_server,&msg,sizeof(struct MSG)); 
//		if(RT_EOK ==err)
//		{
////			rt_kprintf("->send2 msg:%02x %s\n",msg.head,msg.msg_body);
//		}
//		else
//		{
////			rt_kprintf("send2 msg failed:%d\n",err);
//		}

//		rt_thread_delay(2);
//	}
//}

#ifdef RT_USING_UART1	
static rt_err_t uart1_input(rt_device_t dev, rt_size_t size)
{
		uart1_rx_size = size;

		rt_sem_release(&uart1_rx_sem);

    return RT_EOK;
}
#endif

#ifdef RT_USING_UART3	
static rt_err_t uart3_input(rt_device_t dev, rt_size_t size)
{
		uart3_rx_size = size;

		rt_sem_release(&uart3_rx_sem);

    return RT_EOK;
}
#endif

int app_broadcast(void)
{
	rt_err_t err;
	
	rt_sem_init(&uart1_rx_sem, "uart1rx", 0, 0);
	rt_sem_init(&uart3_rx_sem, "uart3rx", 0, 0);

	rt_broadcast_server_init(&s_broadcast_server);
	rt_broadcast_server_init(&s_broadcast_server2);

	///////////////////////////////////////////////
	err =rt_broadcast_client_init(&s_client_1,
								"s_bd_a",
								&s_listener1_mq_buffer[0],
								sizeof(struct MSG),
								sizeof(s_listener1_mq_buffer),
								RT_IPC_FLAG_FIFO);
	if(err !=RT_EOK)
	{
		rt_kprintf("rt_broadcast_client1_init failed:%d\n",err);
		return err;
	}

	rt_broadcast_client_regist(&s_broadcast_server,&s_client_1);

	err = rt_thread_init(&s_client_entry1,
								"s_bd_a",
								broadcast_thread_entry_rev1, RT_NULL,
								s_client_entry1_stack,
								sizeof(s_client_entry1_stack),								
								RT_THREAD_PRIORITY_MAX/3+1, 20);

	if (err == RT_EOK)
	{
		rt_thread_startup(&s_client_entry1);
//		rt_kprintf("broadcast_thread_entry_rev1 startup ok!\n");
	}
	else
	{
		rt_kprintf("broadcast_thread_entry_rev1 startup failed!\n");
	}

	///////////////////////////////////////////////
	err =rt_broadcast_client_init(&s_client_2,
								"s_bd_b",
								&s_listener2_mq_buffer[0],
								sizeof(struct MSG),
								sizeof(s_listener2_mq_buffer),
								RT_IPC_FLAG_FIFO);
	if(err !=RT_EOK)
	{
		rt_kprintf("rt_broadcast_client2_init failed:%d\n",err);
		return err;
	}

	rt_broadcast_client_regist(&s_broadcast_server,&s_client_2);

	err = rt_thread_init(&s_client_entry2,
								"s_bd_b",
								broadcast_thread_entry_rev2, RT_NULL,
								s_client_entry2_stack,
								sizeof(s_client_entry2_stack),								
								RT_THREAD_PRIORITY_MAX/3+1, 20);

	if (err == RT_EOK)
	{
		rt_thread_startup(&s_client_entry2);
//		rt_kprintf("broadcast_thread_entry_rev2 startup ok!\n");
	}
	else
	{
		rt_kprintf("broadcast_thread_entry_rev2 startup failed!\n");
	}

	///////////////////////////////////////////////
	err =rt_broadcast_client_init(&s_client_3,
								"s_bd_c",
								&s_listener3_mq_buffer[0],
								sizeof(struct MSG),
								sizeof(s_listener3_mq_buffer),
								RT_IPC_FLAG_FIFO);
	if(err !=RT_EOK)
	{
		rt_kprintf("rt_broadcast_client3_init failed:%d\n",err);
		return err;
	}

	rt_broadcast_client_regist(&s_broadcast_server2,&s_client_3);

	err = rt_thread_init(&s_client_entry3,
								"s_bd_c",
								broadcast_thread_entry_rev3, RT_NULL,
								s_client_entry3_stack,
								sizeof(s_client_entry3_stack),								
								RT_THREAD_PRIORITY_MAX/3+1, 20);

	if (err == RT_EOK)
	{
		rt_thread_startup(&s_client_entry3);
//		rt_kprintf("broadcast_thread_entry_rev3 startup ok!\n");
	}
	else
	{
		rt_kprintf("broadcast_thread_entry_rev3 startup failed!\n");
	}
	
//	///////////////////////////////////////////////
//	err =rt_broadcast_client_init(&s_client_4,
//								"s_ashell",
//								&s_listener4_mq_buffer[0],
//								sizeof(struct MSG),
//								sizeof(s_listener4_mq_buffer),
//								RT_IPC_FLAG_FIFO);
//	if(err !=RT_EOK)
//	{
//		rt_kprintf("rt_broadcast_client4_init failed:%d\n",err);
//		return err;
//	}

//	rt_broadcast_client_regist(&s_broadcast_server2,&s_client_4);

//	err = rt_thread_init(&s_client_entry4,
//								"s_ashell",
//								broadcast_thread_entry_rev4, RT_NULL,
//								s_client_entry4_stack,
//								sizeof(s_client_entry4_stack),								
//								RT_THREAD_PRIORITY_MAX/3+1, 20);

//	if (err == RT_EOK)
//	{
//		rt_thread_startup(&s_client_entry4);
////		rt_kprintf("broadcast_thread_entry_rev4 startup ok!\n");
//	}
//	else
//	{
//		rt_kprintf("broadcast_thread_entry_rev4 startup failed!\n");
//	}	
//	
//	///////////////////////////////////////////////
//	err =rt_broadcast_client_init(&s_client_5,
//								"s_bshell",
//								&s_listener5_mq_buffer[0],
//								sizeof(struct MSG),
//								sizeof(s_listener5_mq_buffer),
//								RT_IPC_FLAG_FIFO);
//	if(err !=RT_EOK)
//	{
//		rt_kprintf("rt_broadcast_client5_init failed:%d\n",err);
//		return err;
//	}

//	rt_broadcast_client_regist(&s_broadcast_server2,&s_client_5);

//	err = rt_thread_init(&s_client_entry5,
//								"s_bshell",
//								broadcast_thread_entry_rev5, RT_NULL,
//								s_client_entry5_stack,
//								sizeof(s_client_entry5_stack),								
//								RT_THREAD_PRIORITY_MAX/3+1, 20);

//	if (err == RT_EOK)
//	{
//		rt_thread_startup(&s_client_entry5);
////		rt_kprintf("broadcast_thread_entry_rev5 startup ok!\n");
//	}
//	else
//	{
//		rt_kprintf("broadcast_thread_entry_rev5 startup failed!\n");
//	}		
	
	
//	///////////////////////////////////////////////
//	err =rt_broadcast_client_init(&s_client_6,
//								"s_cshell",
//								&s_listener6_mq_buffer[0],
//								sizeof(struct MSG),
//								sizeof(s_listener6_mq_buffer),
//								RT_IPC_FLAG_FIFO);
//	if(err !=RT_EOK)
//	{
//		rt_kprintf("rt_broadcast_client6_init failed:%d\n",err);
//		return err;
//	}

//	rt_broadcast_client_regist(&s_broadcast_server,&s_client_6);

//	err = rt_thread_init(&s_client_entry6,
//								"s_cshell",
//								broadcast_thread_entry_rev6, RT_NULL,
//								s_client_entry6_stack,
//								sizeof(s_client_entry6_stack),								
//								RT_THREAD_PRIORITY_MAX/3+1, 20);

//	if (err == RT_EOK)
//	{
//		rt_thread_startup(&s_client_entry6);
////		rt_kprintf("broadcast_thread_entry_rev6 startup ok!\n");
//	}
//	else
//	{
//		rt_kprintf("broadcast_thread_entry_rev6 startup failed!\n");
//	}			
	
//	///////////////////////////////////////////////
//	err =rt_broadcast_client_init(&s_client_7,
//								"s_pc",
//								&s_listener7_mq_buffer[0],
//								sizeof(struct MSG),
//								sizeof(s_listener7_mq_buffer),
//								RT_IPC_FLAG_FIFO);
//	if(err !=RT_EOK)
//	{
//		rt_kprintf("rt_broadcast_client7_init failed:%d\n",err);
//		return err;
//	}

//	rt_broadcast_client_regist(&s_broadcast_server,&s_client_7);

//	err = rt_thread_init(&s_client_entry7,
//								"s_pc",
//								broadcast_thread_entry_rev7, RT_NULL,
//								s_client_entry7_stack,
//								sizeof(s_client_entry7_stack),								
//								RT_THREAD_PRIORITY_MAX/3+1, 20);

//	if (err == RT_EOK)
//	{
//		rt_thread_startup(&s_client_entry7);
////		rt_kprintf("broadcast_thread_entry_rev7 startup ok!\n");
//	}
//	else
//	{
//		rt_kprintf("broadcast_thread_entry_rev7 startup failed!\n");
//	}			


	///////////////////////////////////////////////
	err =rt_thread_init(&s_send_entry1,"send1",
								broadcast_thread_entry_send1, RT_NULL,
								s_send1_entry_stack,sizeof(s_send1_entry_stack), RT_THREAD_PRIORITY_MAX/3+1, 20);

	if (err ==RT_EOK)
	{
		rt_thread_startup(&s_send_entry1);
//		rt_kprintf("broadcast_thread_entry_send1 startup ok!\n");
	}
	else
	{
		rt_kprintf("broadcast_thread_entry_send1 startup failed!\n");
	}

//	///////////////////////////////////////////////
//	err =rt_thread_init(&s_send_entry2,"send2",
//								broadcast_thread_entry_send2, RT_NULL,
//								s_send2_entry_stack,sizeof(s_send2_entry_stack), RT_THREAD_PRIORITY_MAX/3+1, 20);

//	if (err ==RT_EOK)
//	{
//		rt_thread_startup(&s_send_entry2);
////		rt_kprintf("broadcast_thread_entry_send2 startup ok!\n");
//	}
//	else
//	{
//		rt_kprintf("broadcast_thread_entry_send2 startup failed!\n");
//	}
	
	///////////////////////////////////////////////
	err =rt_thread_init(&s_send_entry3,"send3",
								broadcast_thread_entry_send3, RT_NULL,
								s_send3_entry_stack,sizeof(s_send3_entry_stack), RT_THREAD_PRIORITY_MAX/3+1, 20);

	if (err ==RT_EOK)
	{
		rt_thread_startup(&s_send_entry3);
//		rt_kprintf("broadcast_thread_entry_send3 startup ok!\n");
	}
	else
	{
		rt_kprintf("broadcast_thread_entry_send3 startup failed!\n");
	}	

	
//  rt_memset(app_uart1_rx_fifo.buffer, 0, BUFSIZE);
//	app_uart1_rx_fifo.put_index = 0;
//	app_uart1_rx_fifo.get_index = 0;

#ifdef RT_USING_UART1	
	device = rt_device_find("uart1");

	if (device != RT_NULL)
	{
			rt_device_set_rx_indicate(device, uart1_input);
			rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
	}
#endif
	
#ifdef RT_USING_UART3		
	device3 = rt_device_find("uart3");

	if (device3 != RT_NULL)
	{
			rt_device_set_rx_indicate(device3, uart3_input);
			rt_device_open(device3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
	}
#endif	
	return 0;
}
