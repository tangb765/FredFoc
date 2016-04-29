/******************************************************************************/
/*                                                                            */
/*                                 TLE5012.c                                  */
/*                     Dreamer Rongfei.Deng 2015.9.24                         */
/*                                                                            */
/******************************************************************************/


/*============================================================================*/
/*                               Header include                               */
/*============================================================================*/
#include <board.h>
#ifdef RT_USING_FINSH
#include "finsh.h"
#endif
#include "app_config.h"
#include "TLE5012.h"
/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
#define SPI_Module_CS2_LOW()      GPIO_ResetBits(GPIOA, GPIO_Pin_4) //3
#define SPI_Module_CS2_HIGH()     GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_READ_WRITE_B          0x4000
#define SPI_DUMMY_BYTE            0xffff


/*============================================================================*/
/*                              Global variables                              */
/*============================================================================*/
SPI_InitTypeDef SPI_InitStructure; 


/*============================================================================*/
/*                             Function definition                            */
/*============================================================================*/
static void spi_enc(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

	// spi clk , mosi , miso pin config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// spi cs pin config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_Module_CS2_HIGH(); 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//SPI_Direction_2Lines_FullDuplex  SPI_Direction_1Line_Rx

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;//SPI_DataSize_8b

	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 

	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//SPI_BaudRatePrescaler_256;  //APB2 CLK 72M/256 = 281.25khz; SPI_BaudRatePrescaler_128  SPI_BaudRatePrescaler_64

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;// SPI_FirstBit_LSB;

	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);

//	SPI_SSOutputCmd(SPI1, ENABLE);

	SPI_Cmd(SPI1, ENABLE);
}

static rt_uint16_t SPI_Sensor3_Module_SendByte(rt_uint16_t byte)
{
	rt_uint16_t out;
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
	SPI_Module_CS2_LOW();
	
	SPI_I2S_ReceiveData(SPI1);
	
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, byte);
	
	/* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	out = SPI_I2S_ReceiveData(SPI1);
	
//	  /* Wait to receive a byte */
//  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
//	out[1] = SPI_I2S_ReceiveData(SPI1);
	
//	  /* Wait to receive a byte */
//  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
//	out[2] = SPI_I2S_ReceiveData(SPI1);

	SPI_Cmd(SPI1, DISABLE);
	SPI_Module_CS2_HIGH();
	
	
  /* Return the byte read from the SPI bus */
  return out;
}

#define STAT_REGISTER    0x8001
#define AVAL_REGISTER    0x8021
#define ASPD_REGISTER    0x8032


rt_uint16_t SPI_Sensor_Module_Read(rt_uint16_t regID)
{
  rt_uint16_t Temp;

	Temp = SPI_Sensor3_Module_SendByte(regID);

  return Temp;
}

static rt_bool_t isInit = RT_FALSE;

void TLE5012_init(void)
{
	if(isInit) return;
	
	spi_enc();
	
	isInit = RT_TRUE;
}

rt_int16_t sdasdsad=0;
rt_int16_t TLE5012_Read(void)
{
	sdasdsad = 0x7fff - (SPI_Sensor_Module_Read(AVAL_REGISTER))&0x7fff;
	return sdasdsad;
}


static volatile rt_uint16_t Encoder_pos = 0;
void TLE5012_ResetEncoder(void)
{
	configs.encoder_align_pos = 0x7fff -  (SPI_Sensor_Module_Read(AVAL_REGISTER))&0x7fff;
	configs.Isvalid = configs.Isvalid | 0x8000;
}

rt_int16_t TLE5012_Aligned(void)
{
	Encoder_pos = 0x7fff - (SPI_Sensor_Module_Read(AVAL_REGISTER))&0x7fff; //ANGLECOM_REGISTER
	Encoder_pos = (Encoder_pos - configs.encoder_align_pos)&0x7fff;
	
	return Encoder_pos;
}


rt_int16_t TLE5012_Postion(void)
{
	return  Encoder_pos;
}



