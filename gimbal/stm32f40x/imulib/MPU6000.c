#include "MPU6000.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "app_protocol.h"
#include "stm32f4xx.h"
#include "hrt.h"
#include <drivers/spi.h>
#include "Attitude.h"


#define IMU_GYRO_FS_CFG       MPU6500_GYRO_FS_2000//MPU6500_GYRO_FS_2000//MPU6500_GYRO_FS_1000//MPU6500_GYRO_FS_250//MPU6500_GYRO_FS_1000
#define IMU_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000//MPU6500_DEG_PER_LSB_2000//MPU6500_DEG_PER_LSB_1000//MPU6500_DEG_PER_LSB_250//MPU6500_DEG_PER_LSB_1000
#define IMU_ACCEL_FS_CFG      MPU6500_ACCEL_FS_4//MPU6500_ACCEL_FS_16//MPU6500_ACCEL_FS_8//MPU6500_ACCEL_FS_2//MPU6500_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     MPU6500_G_PER_LSB_4//MPU6500_G_PER_LSB_16//MPU6500_G_PER_LSB_8//MPU6500_G_PER_LSB_2//MPU6500_G_PER_LSB_8
#define IMU_1G_RAW            (int16_t)(1.0 / IMU_G_PER_LSB_CFG)


struct rt_spi_device *mpu6500_dev = RT_NULL;
static void MPU6500_ReadReg(uint8_t reg, uint8_t *rddata)
{
	rt_uint8_t send_buffer[2];
	rt_uint8_t recv_buffer[2];
	
	RT_ASSERT(mpu6500_dev != RT_NULL);

    send_buffer[0] = (0x80 | reg);
    send_buffer[1] = (0x80 | reg);
	rt_spi_transfer(mpu6500_dev, send_buffer, recv_buffer, 2);
	
	*rddata = recv_buffer[1];
	
}

static void MPU6500_ReadMulti(uint8_t reg, uint8_t *buff, uint8_t num)
{
	rt_uint8_t send_data = (0x80 | reg);
	RT_ASSERT(mpu6500_dev != RT_NULL);
	
	rt_spi_send_then_recv(mpu6500_dev,&send_data,1,buff,num);
}

static void MPU6500_WriteReg(uint8_t reg, uint8_t wrdata)
{
	rt_uint8_t send_buffer[2];
	rt_uint8_t recv_buffer[2];
	
	RT_ASSERT(mpu6500_dev != RT_NULL);

	send_buffer[0] = reg;
	send_buffer[1] = wrdata;
	rt_spi_transfer(mpu6500_dev, send_buffer, recv_buffer, 2);
}

float MPU6500_GetFullScaleGyroDPL(void)
{
	uint8_t data;
	float range;
	
	MPU6500_ReadReg(MPU6500_RA_GYRO_CONFIG, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_DEG_PER_LSB_250;
			break;
		case 1:
			range = MPU6500_DEG_PER_LSB_500;
			break;
		case 2:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
		case 3:
			range = MPU6500_DEG_PER_LSB_2000;
			break;
		default:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
	}
	
	return range;
}

float MPU6500_GetFullScaleAccelGPL(void)
{
	uint8_t data;
	float range;
	
	MPU6500_ReadReg(MPU6500_RA_ACCEL_CONFIG, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_G_PER_LSB_2;
			break;
		case 1:
			range = MPU6500_G_PER_LSB_4;
			break;
		case 2:
			range = MPU6500_G_PER_LSB_8;
			break;
		case 3:
			range = MPU6500_G_PER_LSB_16;
			break;
		default:
			range = MPU6500_G_PER_LSB_8;
			break;
	}
	
	return range;
}



rt_uint8_t mpu6500GetDeviceID(void)
{
	uint8_t id = 0;
	MPU6500_ReadReg(MPU6500_RA_WHO_AM_I, &id);
  return id;
}


void mpu6500Reset(void)
{
	MPU6500_WriteReg(MPU6500_RA_PWR_MGMT_1, (1<<MPU6500_PWR1_DEVICE_RESET_BIT));
}

void mpu6500SignalPathReset(void)
{
	MPU6500_WriteReg(MPU6500_RA_SIGNAL_PATH_RESET, ((1<<MPU6500_PATHRESET_GYRO_RESET_BIT)|(1<<MPU6500_PATHRESET_ACCEL_RESET_BIT)|(1<<MPU6500_PATHRESET_TEMP_RESET_BIT)));
}

void mpu6500SetClockSource(uint8_t source)
{
	MPU6500_WriteReg(MPU6500_RA_PWR_MGMT_1, source<<(MPU6500_PWR1_CLKSEL_BIT-MPU6500_PWR1_CLKSEL_LENGTH+1));
}

void mpu6500SensorsEnable(uint8_t range)
{
	MPU6500_WriteReg(MPU6500_RA_PWR_MGMT_2, range);
}

void mpu6500DisableI2C_IF(rt_bool_t enabled)
{
	rt_uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_USER_CTRL,&data);
	if(enabled == RT_TRUE)
		MPU6500_WriteReg(MPU6500_RA_USER_CTRL, data | (1<<MPU6500_USERCTRL_I2C_IF_DIS_BIT));
	else
		MPU6500_WriteReg(MPU6500_RA_USER_CTRL, data | (0<<MPU6500_USERCTRL_I2C_IF_DIS_BIT));
}

void mpu6500SetFullScaleGyroRange(uint8_t range)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_GYRO_CONFIG,&data);
	MPU6500_WriteReg(MPU6500_RA_GYRO_CONFIG, data | (range<<(MPU6500_GCONFIG_FS_SEL_BIT-MPU6500_GCONFIG_FS_SEL_LENGTH+1)));
}

void mpu6500SetGC_FCHOICE_B(uint8_t range)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_GYRO_CONFIG,&data);
	MPU6500_WriteReg(MPU6500_RA_GYRO_CONFIG, data | (range<<(MPU6500_GCONFIG_FCHOICE_B_BIT-MPU6500_GCONFIG_FCHOICE_B_LENGTH+1)));
}

void mpu6500SetGDLPFMode(uint8_t bandwidth)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_GYRO_CONFIG,&data);
	MPU6500_WriteReg(MPU6500_RA_GYRO_CONFIG, data | (bandwidth<<(MPU6500_CONFIG_GDLPF_CFG_BIT-MPU6500_CONFIG_GDLPF_CFG_LENGTH+1)));
}

void mpu6500SetFullScaleAccelRange(uint8_t range)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_ACCEL_CONFIG,&data);
	MPU6500_WriteReg(MPU6500_RA_ACCEL_CONFIG, data | (range<<(MPU6500_ACONFIG_AFS_SEL_BIT-MPU6500_ACONFIG_AFS_SEL_LENGTH+1)));
}


void mpu6500SetAC2_FCHOICE_B(uint8_t range)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_ACCEL_CONFIG2,&data);
	MPU6500_WriteReg(MPU6500_RA_ACCEL_CONFIG2, data | (range<<(MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BIT-MPU6500_ACONFIG2_ACCEL_FCHOICE_B_LENGTH+1)));
}

void mpu6500SetADLPFMode(uint8_t bandwidth)
{
	uint8_t data;
	MPU6500_ReadReg(MPU6500_RA_ACCEL_CONFIG2,&data);
	MPU6500_WriteReg(MPU6500_RA_ACCEL_CONFIG2, data | (bandwidth<<(MPU6500_ACONFIG2_A_DLPF_CFG_BIT-MPU6500_ACONFIG2_A_DLPF_CFG_LENGTH+1)));
}

void mpu6500SetRate(uint8_t rate)
{
	MPU6500_WriteReg(MPU6500_RA_SMPLRT_DIV, rate);
}

rt_err_t mpu6500_connect_test(void)
{
  if (mpu6500GetDeviceID() == 0x70)
  {
//		output_flag = IMU;
//    rt_kprintf("MPU6500 SPI connection [OK].\r\n");
		return RT_EOK;
  }
  else
  {
		output_flag = CLOSE;
    rt_kprintf("MPU6500 SPI connection [FAIL].\r\n");
		return RT_ERROR;
  }
}


rt_err_t MPU6500_Init(void)
{
	struct rt_spi_configuration cfg;
	mpu6500_dev = (struct rt_spi_device *)rt_device_find("mpu6500");
	//rt_device_open(mpu6500_dev,RT_DEVICE_OFLAG_RDWR);
	
	cfg.data_width = 8;
	cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */

	//SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M, ...
	cfg.max_hz = 1000*1000; /* 11000kbit/s */ 
	rt_spi_configure(mpu6500_dev, &cfg);
	
	if(mpu6500_connect_test() == RT_ERROR)
	{
		return RT_ERROR;
	}

/* 1. Set H_RESET = 1 (register PWR_MGMT_1)
 * The reset value is 0x00 for all registers other than the registers below, 
 * also the self-test registers contain preprogrammed values and will not be 0x00 after reset.
 * Register 107 (0x01) Power Management 1
 * Register 117 (0x70) WHO_AM_I
**/
  mpu6500Reset();
	delay_ms(100);

	
//2. Set GYRO_RST = ACCEL_RST = TEMP_RST = 1 (register SIGNAL_PATH_RESET)
	mpu6500SignalPathReset();
	delay_ms(100);

//3.config modes and Clock Source,PLL if ready, else use the Internal oscillator(register PWR_MGMT_1)
	mpu6500SetClockSource(MPU6500_CLOCK_PLL);
//  mpu6500SetSleepEnabled(FALSE);
//  mpu6500SetTempSensorEnabled(TRUE);
//  mpu6500SetIntEnabled(FALSE);
	
//4.All sensors on(register PWR_MGMT_2)	
	mpu6500SensorsEnable(MPU6500_PWR2_ENABLE_ALL_SENSORS);
//	mpu6500SensorsEnable(MPU6500_PWR2_STBY_XA);
	
//5.config USER_CTRL,SPI mode only(register USER_CTRL)		
	mpu6500DisableI2C_IF(RT_TRUE);

//6.Gyro configs:Gyro Full Scale,Gyro Sample Rate,DLPF_BW
  mpu6500SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
	mpu6500SetGC_FCHOICE_B(MPU6500_GCONFIG_FCHOICE_B_BW_xxxx);
	mpu6500SetGDLPFMode(MPU6500_GDLPF_BW_250);

//7.Accel configs:Accel Full Scale,Accel Sample Rate,DLPF_BW
  mpu6500SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);
	mpu6500SetAC2_FCHOICE_B(MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BW_xxxx);
	mpu6500SetADLPFMode(MPU6500_ADLPF_BW_10);

//8.set ODR:Gyro FCHOICE_B = 0 0,GDLPF_BW=250 RATRE=1KHZ,so 1000 / (1 + 1) = 500Hz;Accel FCHOICE_B = 0,ADLPF_BW=460 RATRE=1KHZ,so 1000 / (1 + 1) = 500Hz;
  mpu6500SetRate(1);
	
	cfg.max_hz = 1000*20000; /* 11000kbit/s */ 
	rt_spi_configure(mpu6500_dev, &cfg);
	return RT_EOK;
}

void MPU6500_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
						int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t buffer[14];
	
	MPU6500_ReadMulti(MPU6500_RA_ACCEL_XOUT_H, buffer, 14);
	*ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	*az = (((int16_t) buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}
