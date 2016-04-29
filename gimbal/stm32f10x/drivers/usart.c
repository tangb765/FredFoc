/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2013-05-13     aozima       update for kehong-lingtai.
 * 2015-01-31     armink       make sure the serial transmit complete in putc()
 */

#include "stm32f10x.h"
#include "usart.h"
#include "board.h"

#include <rtdevice.h>

/* config */
#define RT_USING_UART1_DMA
#define RT_USING_UART3_DMA
#define USE_USART1_REMAP

#ifdef USE_USART1_REMAP
/* USART1 */
#define UART1_GPIO_TX        GPIO_Pin_6
#define UART1_GPIO_RX        GPIO_Pin_7
#define UART1_GPIO           GPIOB
#define UART1_TX_DMA         DMA1_Channel4
#define UART1_RX_DMA         DMA1_Channel5
#else
/* USART1 */
#define UART1_GPIO_TX        GPIO_Pin_9
#define UART1_GPIO_RX        GPIO_Pin_10
#define UART1_GPIO           GPIOA
#define UART1_TX_DMA         DMA1_Channel4
#define UART1_RX_DMA         DMA1_Channel5
#endif


/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_TX        GPIO_Pin_10
#define UART3_GPIO_RX        GPIO_Pin_11
#define UART3_GPIO           GPIOB
#define UART3_TX_DMA         DMA1_Channel2
#define UART3_RX_DMA         DMA1_Channel3

#define BUFSIZE  64

struct rt_uart_rx_fifo
{
	/* software fifo */
	rt_uint8_t buffer[BUFSIZE];
//	rt_uint8_t *buffer;

	rt_uint16_t put_index, get_index;
};

struct rt_uart_tx_fifo
{
	/* software fifo */
	rt_uint8_t buffer[BUFSIZE];
//	rt_uint8_t *buffer;

	rt_uint16_t put_index, get_index;
};

#if defined(RT_USING_UART1)
rt_uint8_t txbuf[BUFSIZE];
rt_uint8_t rxbuf[BUFSIZE];
//rt_uint8_t rxbuf_ref[BUFSIZE];
//uint16_t rxbuf_len = 0;
//struct rt_uart_rx_fifo uart1_rx_fifo;
struct rt_uart_tx_fifo uart1_tx_fifo;
#endif

#if defined(RT_USING_UART3)
rt_uint8_t tx3buf[BUFSIZE];
rt_uint8_t rx3buf[BUFSIZE];
//rt_uint8_t rx3buf_ref[BUFSIZE];
//uint16_t rx3buf_len = 0;
//struct rt_uart_rx_fifo uart3_rx_fifo;
struct rt_uart_tx_fifo uart3_tx_fifo;
#endif

/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
    DMA_Channel_TypeDef *tx_dma_channel;
    IRQn_Type DMA_irq;
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
		DMA1_Channel4,
		DMA1_Channel4_IRQn,	
};
struct rt_serial_device serial1;
#endif

#if defined(RT_USING_UART3)
/* UART3 device driver structure */
struct stm32_uart uart3 =
{
    USART3,
    USART3_IRQn,
		DMA1_Channel2,
		DMA1_Channel2_IRQn,
};
struct rt_serial_device serial3;
#endif

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart *uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

		if (cfg->baud_rate == BAUD_RATE_2400)
					USART_InitStructure.USART_BaudRate = 2400;
		else if (cfg->baud_rate == BAUD_RATE_4800)
					USART_InitStructure.USART_BaudRate = 4800;
		else if (cfg->baud_rate == BAUD_RATE_9600)
					USART_InitStructure.USART_BaudRate = 9600;
		else if (cfg->baud_rate == BAUD_RATE_38400)
					USART_InitStructure.USART_BaudRate = 38400;
		else if (cfg->baud_rate == BAUD_RATE_57600)
					USART_InitStructure.USART_BaudRate = 57600;
		else if (cfg->baud_rate == BAUD_RATE_115200)
					USART_InitStructure.USART_BaudRate = 115200;
		else if (cfg->baud_rate == BAUD_RATE_230400)
					USART_InitStructure.USART_BaudRate = 230400;
		else if (cfg->baud_rate == BAUD_RATE_460800)
					USART_InitStructure.USART_BaudRate = 460800;
		else if (cfg->baud_rate == BAUD_RATE_921600)
					USART_InitStructure.USART_BaudRate = 921600;

    if (cfg->data_bits == DATA_BITS_8)
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else if (cfg->data_bits == DATA_BITS_9)
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;

    if (cfg->parity == PARITY_NONE)
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    else if (cfg->parity == PARITY_ODD)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    }
    else if (cfg->parity == PARITY_EVEN)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    uart->uart_device->DR = c;
    while (!(uart->uart_device->SR & USART_FLAG_TC));

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

static void uart1_tx_idle_hook()
{
#if defined(RT_USING_UART1)
	rt_base_t level;	
	
	if (uart1_tx_fifo.get_index != uart1_tx_fifo.put_index && (uart1.tx_dma_channel->CCR & (uint32_t)DMA_CCR1_EN) == 0)
	{
		rt_size_t count=0;
		rt_size_t length=0; 					
		rt_size_t ch=0; 
		
		if (uart1_tx_fifo.get_index < uart1_tx_fifo.put_index)
		{
			count = uart1_tx_fifo.put_index - uart1_tx_fifo.get_index;
		}
		else
		{
			count = BUFSIZE - uart1_tx_fifo.get_index + uart1_tx_fifo.put_index;
		}
		
		length = count;
		
		/* disable interrupt */ 
		level = rt_hw_interrupt_disable();
		
		while (count)
		{			
				if (uart1_tx_fifo.get_index != uart1_tx_fifo.put_index)
				{
						txbuf[ch] = uart1_tx_fifo.buffer[uart1_tx_fifo.get_index];
						uart1_tx_fifo.get_index += 1;
						ch++;
						if (uart1_tx_fifo.get_index >= BUFSIZE) 
							uart1_tx_fifo.get_index = 0;
				}
				else
				{
				
				}

				count --;
		}
		
		/* enable interrupt */
		rt_hw_interrupt_enable(level);		
		
		
		/* set buffer address */
		uart1.tx_dma_channel->CMAR = (rt_uint32_t) txbuf;
		/* set size */
		uart1.tx_dma_channel->CNDTR = length;
		/* enable DMA */
		DMA_Cmd(uart1.tx_dma_channel, ENABLE);	
	}
#endif
}

static void uart3_tx_idle_hook()
{
#if defined(RT_USING_UART3)
	rt_base_t level;	
	
	if (uart3_tx_fifo.get_index != uart3_tx_fifo.put_index && (uart3.tx_dma_channel->CCR & (uint32_t)DMA_CCR1_EN) == 0)
	{
		rt_size_t count=0;
		rt_size_t length=0; 					
		rt_size_t ch=0; 
		
		if (uart3_tx_fifo.get_index < uart3_tx_fifo.put_index)
		{
			count = uart3_tx_fifo.put_index - uart3_tx_fifo.get_index;
		}
		else
		{
			count = BUFSIZE - uart3_tx_fifo.get_index + uart3_tx_fifo.put_index;
		}
		
		length = count;
		
		/* disable interrupt */ 
		level = rt_hw_interrupt_disable();
		
		while (count)
		{			
				if (uart3_tx_fifo.get_index != uart3_tx_fifo.put_index)
				{
						tx3buf[ch] = uart3_tx_fifo.buffer[uart3_tx_fifo.get_index];
						uart3_tx_fifo.get_index += 1;
						ch++;
						if (uart3_tx_fifo.get_index >= BUFSIZE) 
							uart3_tx_fifo.get_index = 0;
				}
				else
				{
					
				}

				count --;
		}
		
		/* enable interrupt */
		rt_hw_interrupt_enable(level);
		
		/* set buffer address */
		uart3.tx_dma_channel->CMAR = (rt_uint32_t) tx3buf;
		/* set size */
		uart3.tx_dma_channel->CNDTR = length;
		/* enable DMA */
		DMA_Cmd(uart3.tx_dma_channel, ENABLE);	
	}
#endif	
}

void uart_tx_idle_hook()
{
//	rt_base_t level;
	
//	/* disable interrupt */ 
//	level = rt_hw_interrupt_disable();
	
	uart1_tx_idle_hook();
	uart3_tx_idle_hook();
	
//	/* enable interrupt */
//	rt_hw_interrupt_enable(level);
}


rt_size_t stm32_dma_transmit(struct rt_serial_device *serial,const rt_uint8_t *buf, rt_size_t size, int direction)
{
    struct stm32_uart *uart;
		rt_base_t level;	
		rt_size_t length;
	
	
    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
	
	  /* disable interrupt */ 
		level = rt_hw_interrupt_disable();

#if defined(RT_USING_UART1)
		if(uart == &uart1)
		{
			uint16_t len = 0; 
			uint16_t len_i = 0; 
			
			if (direction == RT_SERIAL_DMA_TX)
			{
				len = size;
				while(len>0)
				{
					if (buf[len_i]=='\n' && buf[len_i-1]!='\r')
					{
						uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = '\r';
						uart1_tx_fifo.put_index += 1;
						if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
						
						/* if the next position is read index, discard this 'read char' */
						if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
						{
								uart1_tx_fifo.get_index += 1;
								if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
						}					
					}
					
					uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = buf[len_i];
					uart1_tx_fifo.put_index += 1;
					if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
					
					/* if the next position is read index, discard this 'read char' */
					if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
					{
							uart1_tx_fifo.get_index += 1;
							if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
					}
					len--;			
					len_i++;
				}
				
				rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);	

				length = size;
			}
			else
			{
//				rt_size_t ch = 0;

				length = size; 
//				while (size)
//				{			
//						if (uart1_rx_fifo.get_index != uart1_rx_fifo.put_index)
//						{
//								rxbuf_ref[ch] = uart1_rx_fifo.buffer[uart1_rx_fifo.get_index];
//								uart1_rx_fifo.get_index += 1;
//								ch++;
//								if (uart1_rx_fifo.get_index >= BUFSIZE) 
//									uart1_rx_fifo.get_index = 0;
//						}
//						else
//						{
//								/* no data, enable interrupt and break out */
//								length = 0;
//						}

//						size --;
//				}
//				
//				if (length != size) rt_memcpy((void*)buf,rxbuf_ref,length - size);

//				length = length - size;
				
				rt_memcpy((void*)buf,rxbuf,length);

				DMA_SetCurrDataCounter(UART1_RX_DMA,BUFSIZE);  
				DMA_Cmd(UART1_RX_DMA,ENABLE);  
							
			}
		}
#endif
		
#if defined(RT_USING_UART3)
		if(uart==&uart3)
		{
			uint16_t len = 0; 
			uint16_t len_i = 0; 
			
			if (direction == RT_SERIAL_DMA_TX)
			{
				len = size;
				while(len>0)
				{
					if (buf[len_i]=='\n' && buf[len_i-1]!='\r')
					{
						uart3_tx_fifo.buffer[uart3_tx_fifo.put_index] = '\r';
						uart3_tx_fifo.put_index += 1;
						if (uart3_tx_fifo.put_index >= BUFSIZE) uart3_tx_fifo.put_index = 0;
						
						/* if the next position is read index, discard this 'read char' */
						if (uart3_tx_fifo.put_index == uart3_tx_fifo.get_index)
						{
								uart3_tx_fifo.get_index += 1;
								if (uart3_tx_fifo.get_index >= BUFSIZE) uart3_tx_fifo.get_index = 0;
						}					
					}
					
					uart3_tx_fifo.buffer[uart3_tx_fifo.put_index] = buf[len_i];
					uart3_tx_fifo.put_index += 1;
					if (uart3_tx_fifo.put_index >= BUFSIZE) uart3_tx_fifo.put_index = 0;
					
					/* if the next position is read index, discard this 'read char' */
					if (uart3_tx_fifo.put_index == uart3_tx_fifo.get_index)
					{
							uart3_tx_fifo.get_index += 1;
							if (uart3_tx_fifo.get_index >= BUFSIZE) uart3_tx_fifo.get_index = 0;
					}
					len--;			
					len_i++;
				}
				
				rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);	

				length = size;
			}
			else
			{
//				rt_size_t ch = 0;
				
				length = size; 
//				while (size)
//				{			
//						if (uart3_rx_fifo.get_index != uart3_rx_fifo.put_index)
//						{
//								rx3buf_ref[ch] = uart3_rx_fifo.buffer[uart3_rx_fifo.get_index];
//								uart3_rx_fifo.get_index += 1;
//								ch++;
//								if (uart3_rx_fifo.get_index >= BUFSIZE) 
//									uart3_rx_fifo.get_index = 0;
//						}
//						else
//						{
//								/* no data, enable interrupt and break out */
//								length = 0;
//						}

//						size --;
//				}
//				
//				if (length != size) rt_memcpy((void*)buf,rx3buf_ref,length - size);

//				length = length - size;	
				
				rt_memcpy((void*)buf,rx3buf,length);

				DMA_SetCurrDataCounter(UART3_RX_DMA,BUFSIZE);  
				DMA_Cmd(UART3_RX_DMA,ENABLE);  
						
			}	
		}
#endif
		
		
		/* enable interrupt */
		rt_hw_interrupt_enable(level);
		
		return length;
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
    stm32_dma_transmit,
};

#if defined(RT_USING_UART1)
void USART1_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

		
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
				rt_size_t len = 0; 
			  rt_size_t len_i = 0; 
				
				USART1->SR;  
				USART1->DR;

				DMA_Cmd(UART1_RX_DMA,DISABLE);  
				DMA_ClearFlag(DMA1_FLAG_TC5);  
					
			  len = BUFSIZE - DMA_GetCurrDataCounter(UART1_RX_DMA);
				
				len_i = 0; 
				while(len_i<len)
				{
					if(rxbuf[len_i] == '#')
						break;

					len_i++;
				}
				
				if (len_i>=len)
				{
					len_i = 0; 
					
					while(len_i<len)
					{
						if (rxbuf[len_i]=='\n' && rxbuf[len_i-1]!='\r')
						{
							uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = '\r';
							uart1_tx_fifo.put_index += 1;
							if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
							
							/* if the next position is read index, discard this 'read char' */
							if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
							{
									uart1_tx_fifo.get_index += 1;
									if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
							}					
						}
						
						uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = rxbuf[len_i];
						uart1_tx_fifo.put_index += 1;
						if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
						
						/* if the next position is read index, discard this 'read char' */
						if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
						{
								uart1_tx_fifo.get_index += 1;
								if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
						}	
						len_i++;
					}
					
					DMA_SetCurrDataCounter(UART1_RX_DMA,BUFSIZE);  
					DMA_Cmd(UART1_RX_DMA,ENABLE); 
				}
				else
				{
//					len_i = 0; 
//					while(len_i<len)
//					{
//						uart1_rx_fifo.buffer[uart1_rx_fifo.put_index] = rxbuf[len_i];
//						uart1_rx_fifo.put_index += 1;
//						if (uart1_rx_fifo.put_index >= BUFSIZE) uart1_rx_fifo.put_index = 0;
//						
//						/* if the next position is read index, discard this 'read char' */
//						if (uart1_rx_fifo.put_index == uart1_rx_fifo.get_index)
//						{
//								uart1_rx_fifo.get_index += 1;
//								if (uart1_rx_fifo.get_index >= BUFSIZE) uart1_rx_fifo.get_index = 0;
//						}
//		
//						len_i++;
//					}
					
					if (len>0)
						rt_hw_serial_isr(&serial1, ((len << 8) & (~0xff)) | RT_SERIAL_EVENT_RX_DMADONE);
				}
    }   

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_USING_UART1_DMA
void DMA1_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if(DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#endif /* RT_USING_UART1 */


#if defined(RT_USING_UART3)
void USART3_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart3;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  
    {  
				rt_size_t len = 0; 
			  rt_size_t len_i = 0; 
				
				USART3->SR;  
				USART3->DR;

				DMA_Cmd(UART3_RX_DMA,DISABLE);  
				DMA_ClearFlag(DMA1_FLAG_TC3);  
					
			  len = BUFSIZE - DMA_GetCurrDataCounter(UART3_RX_DMA);
				
				len_i = 0; 
				while(len_i<len)
				{
					if(rx3buf[len_i] == '#')
						break;
					len_i++;
				}
				
				if (len_i>=len)
				{
					len_i = 0; 
					
					while(len_i<len)
					{
						if (rx3buf[len_i]=='\n' && rx3buf[len_i-1]!='\r')
						{
							uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = '\r';
							uart1_tx_fifo.put_index += 1;
							if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
							
							/* if the next position is read index, discard this 'read char' */
							if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
							{
									uart1_tx_fifo.get_index += 1;
									if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
							}					
						}
						
						uart1_tx_fifo.buffer[uart1_tx_fifo.put_index] = rx3buf[len_i];
						uart1_tx_fifo.put_index += 1;
						if (uart1_tx_fifo.put_index >= BUFSIZE) uart1_tx_fifo.put_index = 0;
						
						/* if the next position is read index, discard this 'read char' */
						if (uart1_tx_fifo.put_index == uart1_tx_fifo.get_index)
						{
								uart1_tx_fifo.get_index += 1;
								if (uart1_tx_fifo.get_index >= BUFSIZE) uart1_tx_fifo.get_index = 0;
						}	
						len_i++;
					}
					
					DMA_SetCurrDataCounter(UART3_RX_DMA,BUFSIZE);  
					DMA_Cmd(UART3_RX_DMA,ENABLE); 
				}
				else
				{
//					len_i = 0;
//			
//					while(len_i<len)
//					{
//						uart3_rx_fifo.buffer[uart3_rx_fifo.put_index] = rx3buf[len_i];
//						uart3_rx_fifo.put_index += 1;
//						if (uart3_rx_fifo.put_index >= BUFSIZE) uart3_rx_fifo.put_index = 0;
//						
//						/* if the next position is read index, discard this 'read char' */
//						if (uart3_rx_fifo.put_index == uart3_rx_fifo.get_index)
//						{
//								uart3_rx_fifo.get_index += 1;
//								if (uart3_rx_fifo.get_index >= BUFSIZE) uart3_rx_fifo.get_index = 0;
//						}
//		
//						len_i++;
//					}

	
					if (len>0)
						rt_hw_serial_isr(&serial3, ((len << 8) & (~0xff)) | RT_SERIAL_EVENT_RX_DMADONE);
				}
    }   

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_USING_UART3_DMA
void DMA1_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if(DMA_GetITStatus(DMA1_IT_TC2) != RESET)
    {
        DMA_Cmd(DMA1_Channel2, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#endif /* RT_USING_UART3 */


static void RCC_Configuration(void)
{
#if defined(RT_USING_UART1)
#ifdef USE_USART1_REMAP
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

#ifdef RT_USING_UART1_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
#else
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

#ifdef RT_USING_UART1_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

#endif
#endif /* RT_USING_UART1 */


#if defined(RT_USING_UART3)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#ifdef RT_USING_UART3_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
#endif /* RT_USING_UART3 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#if defined(RT_USING_UART1)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#ifdef USE_USART1_REMAP
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
#endif
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART3 */

}

static void NVIC_Configuration(struct stm32_uart *uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = uart->DMA_irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}


static void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

#ifdef RT_USING_UART1
#ifdef RT_USING_UART1_DMA
    DMA_DeInit(UART1_TX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(UART1_TX_DMA, &DMA_InitStructure);
		DMA_ITConfig(UART1_TX_DMA, DMA_IT_TC | DMA_IT_TE , ENABLE);


    DMA_DeInit(UART1_RX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxbuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFSIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(UART1_RX_DMA, &DMA_InitStructure);
		DMA_Cmd(UART1_RX_DMA, ENABLE);
		
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
#endif
#endif

#ifdef RT_USING_UART3
#ifdef RT_USING_UART3_DMA
    DMA_DeInit(UART3_TX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(UART3_TX_DMA, &DMA_InitStructure);
		DMA_ITConfig(UART3_TX_DMA, DMA_IT_TC | DMA_IT_TE , ENABLE);


    DMA_DeInit(UART3_RX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx3buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFSIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(UART3_RX_DMA, &DMA_InitStructure);
		DMA_Cmd(UART3_RX_DMA, ENABLE);
		
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART3, USART_IT_TC, DISABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
#endif
#endif
}


int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    RCC_Configuration();
    GPIO_Configuration();
		DMA_Configuration();

#ifdef RT_USING_UART1	
    uart = &uart1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                          uart);
	
#ifdef RT_USING_UART1_DMA
//		rt_memset(uart1_rx_fifo.buffer, 0, BUFSIZE);
//		uart1_rx_fifo.put_index = 0;
//		uart1_rx_fifo.get_index = 0;
		
		rt_memset(uart1_tx_fifo.buffer, 0, BUFSIZE);
		uart1_tx_fifo.put_index = 0;
		uart1_tx_fifo.get_index = 0;		

    /* Enable USART1 DMA Tx request */
    USART_DMACmd(USART1, USART_DMAReq_Tx , ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx , ENABLE);  
		
		rt_thread_idle_sethook(uart_tx_idle_hook);		
		
#endif  /* RT_USING_UART1_DMA */

#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART3
    uart = &uart3;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(&uart3);

    /* register UART3 device */
    rt_hw_serial_register(&serial3,
                          "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                          uart);
													
#ifdef RT_USING_UART1_DMA
//		rt_memset(uart3_rx_fifo.buffer, 0, BUFSIZE);
//		uart3_rx_fifo.put_index = 0;
//		uart3_rx_fifo.get_index = 0;
		
		rt_memset(uart3_tx_fifo.buffer, 0, BUFSIZE);
		uart3_tx_fifo.put_index = 0;
		uart3_tx_fifo.get_index = 0;		

    /* Enable USART1 DMA Tx request */
    USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);
		USART_DMACmd(USART3, USART_DMAReq_Rx , ENABLE);  
		
		rt_thread_idle_sethook(uart_tx_idle_hook);		
		
#endif  /* RT_USING_UART1_DMA */
#endif /* RT_USING_UART3 */

    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);
