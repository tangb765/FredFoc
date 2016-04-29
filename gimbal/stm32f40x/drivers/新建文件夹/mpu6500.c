#include "mpu6500.h"
#include <drivers/spi.h>
#include "rtthread.h"
#include "rtdevice.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdbool.h>
#include "hrt.h"
struct rt_spi_device *mpu6500_dev = RT_NULL;

#define    MPU6500_GET_TIME_MS()           hrt_absolute_ms()

#define DEBUG_PRINT(...)

static void MPU6500_DelayMs(uint32_t ms)
{
	uint64_t current_ms;
	current_ms = hrt_absolute_ms();
	
	while(current_ms+ms >= hrt_absolute_ms());
}

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
	
	MPU6500_ReadReg(27, &data);
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
	
	MPU6500_ReadReg(28, &data);
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

void MPU6500_MagMeasureContinu(void)
{
	//SLV0
	MPU6500_WriteReg(37, 0X8C);//AK8975 Address READ
	MPU6500_WriteReg(38, 0X03);//AK8975 HXL
	MPU6500_WriteReg(39, 0x86);//enable		

	//SLV1
	MPU6500_WriteReg(40, 0X0C);//AK8975 Address READ
	MPU6500_WriteReg(41, 0X0A);//AK8975 CNTL1
	MPU6500_WriteReg(42, 0x81);//enable	
	MPU6500_WriteReg(100, 0X11);//16BIT Single measurement mode 
	
	MPU6500_WriteReg(52, 0x04);//I2C_MST_DLY = 4
	MPU6500_WriteReg(103, 0x03);//I2C_SLV0_DLY_EN 
}

uint8_t MPU6500_Check(void)
{
	uint8_t mid = 0;
	
	MPU6500_ReadReg(117, &mid);
	if(mid == 0x71)
		return 0;
	else
		return 1;
}

static bool MPU6500_EvaluateSelfTest(float low, float high, float value, char* string)
{
	if (value < low || value > high)
	{
		return false;
	}
	return true;
}

static uint8_t MPU6500_SelfTest(void)
{
	uint8_t asel,gsel;
	
	int16_t axi16, ayi16, azi16;
	int16_t gxi16, gyi16, gzi16;
	float axf, ayf, azf;
	float gxf, gyf, gzf;
	float axfTst, ayfTst, azfTst;
	float gxfTst, gyfTst, gzfTst;
	float axfDiff, ayfDiff, azfDiff;
	float gxfDiff, gyfDiff, gzfDiff;
	float gRange, aRange;
	uint32_t scrap;
	
	aRange = MPU6500_GetFullScaleAccelGPL();
	gRange = MPU6500_GetFullScaleGyroDPL();

	// First values after startup can be read as zero. Scrap a couple to be sure.
	for (scrap = 0; scrap < 20; scrap++)
	{
		MPU6500_GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
		MPU6500_DelayMs(2);
	}
	// First measurement
	gxf = gxi16 * gRange;
	gyf = gyi16 * gRange;
	gzf = gzi16 * gRange;
	axf = axi16 * aRange;
	ayf = ayi16 * aRange;
	azf = azi16 * aRange;

	// Enable self test
	MPU6500_ReadReg(27, &gsel);
	MPU6500_ReadReg(28, &asel);
	MPU6500_WriteReg(27, 0xE0|gsel);
	MPU6500_WriteReg(28, 0xE0|asel);


	// Wait for self test to take effect
	MPU6500_DelayMs(10);
	// Take second measurement
	MPU6500_GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
	gxfTst = gxi16 * gRange;
	gyfTst = gyi16 * gRange;
	gzfTst = gzi16 * gRange;
	axfTst = axi16 * aRange;
	ayfTst = ayi16 * aRange;
	azfTst = azi16 * aRange;

	// Disable self test
	MPU6500_WriteReg(27, gsel);
	MPU6500_WriteReg(28, asel);

	// Calculate difference
	gxfDiff = gxfTst - gxf;
	gyfDiff = gyfTst - gyf;
	gzfDiff = gzfTst - gzf;
	axfDiff = axfTst - axf;
	ayfDiff = ayfTst - ayf;
	azfDiff = azfTst - azf;

	// Check result
	if (MPU6500_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gxfDiff, "gyro X") &&
	MPU6500_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gyfDiff, "gyro Y") &&
	MPU6500_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gzfDiff, "gyro Z") &&
	MPU6500_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, axfDiff, "acc X") &&
	MPU6500_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, ayfDiff, "acc Y") &&
	MPU6500_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, azfDiff, "acc Z"))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

rt_err_t MPU6500_Init(void)
{
	struct rt_spi_configuration cfg;
	uint8_t id = 0;
	mpu6500_dev = (struct rt_spi_device *)rt_device_find("mpu6500");
	//rt_device_open(mpu6500_dev,RT_DEVICE_OFLAG_RDWR);
	
	cfg.data_width = 8;
	cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
	
	
	//SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M, ...
	cfg.max_hz = 1000*1000; /* 11000kbit/s */ 
	rt_spi_configure(mpu6500_dev, &cfg);
	 
	
	MPU6500_WriteReg(107, 0X80);//Reset
	MPU6500_DelayMs(100);
	MPU6500_WriteReg(107, 0X01);//Clock Source 
	MPU6500_WriteReg(108, 0X00);//Enable Acc & Gyro
	
	MPU6500_WriteReg(56, 0X01);//enabled RAW_RDY_EN Interrupt
	MPU6500_WriteReg(55, 0XB0);//disabled BYPASS  LOW INT
	
	//MPU6500_WriteReg(106, 0X30);//I2C_MST_EN
	//MPU6500_WriteReg(36, 0X4D);//I2C Speed 400 kHz
	
	MPU6500_WriteReg(25, 0X01);//SMPLRT_DIV
	MPU6500_WriteReg(26, 0X01);//Bandwidth = 184Hz, FS=1KHz
	//MPU6500_WriteReg(26, 0X02);//Bandwidth = 92Hz, FS=1KHz
	MPU6500_WriteReg(27, 0X18);//2000 dps 
	//MPU6500_WriteReg(28, 0X10);//8g 
	MPU6500_WriteReg(28, 0X08);//4g 
	MPU6500_WriteReg(29, 0X00);//Bandwidth = 460Hz, FS=1KHz
	
	MPU6500_ReadReg(117, &id);
	
	if(id != 0x70)
	{
	   rt_kprintf("\nmpu6500 not found\r\n");
	}
	
	if(MPU6500_SelfTest())
	{
	    return RT_ERROR;
	}
	
	cfg.max_hz = 1000*20000; /* 11000kbit/s */ 
	rt_spi_configure(mpu6500_dev, &cfg);
	return RT_EOK;
}

void MPU6500_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
						int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t buffer[14];
	
	MPU6500_ReadMulti(59, buffer, 14);
	*ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	*az = (((int16_t) buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}

