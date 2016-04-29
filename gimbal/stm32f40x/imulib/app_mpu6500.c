

#include "stdio.h"
#include "Attitude.h"
#include "AttitudeEKF.h"
#include "MPU6000.h"
#include "hrt.h"
#include "app_config.h"
#include "app_protocol.h"

extern uint64_t varianceSampleTime;
imu_s  imu;

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = RT_FALSE;
  bias->bufHead = bias->buffer;
}


/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[3] = {0};
  int64_t sumSq[3] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;
}


/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = RT_TRUE;
  }
}



/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static rt_bool_t imuFindBiasValue(BiasObj* bias)
{
  rt_bool_t foundBias = RT_FALSE;

  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < hrt_absolute_ms()))
    {
//      varianceSampleTime = hrt_absolute_ms();
			configs.imu.gyro_Bx = mean.x;
			configs.imu.gyro_By = mean.y;
			configs.imu.gyro_Bz = mean.z;
			foundBias = RT_TRUE;
			configs.imu.iscalibed = configs.imu.iscalibed | 0x0100;
    }
  }

  return foundBias;
}


void mpu6500_thread_entry(void* parameter)
{
	rt_int16_t tmp1,tmp2;
	static rt_uint8_t cptcs = 0;
	imu.acc_scale = MPU6500_GetFullScaleAccelGPL();
	imu.gyro_scale = MPU6500_GetFullScaleGyroDPL();
	
	varianceSampleTime = hrt_absolute_ms();
	
	while(1)
	{
		rt_thread_delay( RT_TICK_PER_SECOND/500 ); 
		
		cptcs++;
		
		if(mpu6500_connect_test() == RT_ERROR)
		{
			continue;
		}
		
		imu.timestamp = hrt_absolute_time();
//		MPU6500_GetMotion6(&imu.acc_i16.x,&imu.acc_i16.y,&imu.acc_i16.z,&imu.gyro_i16.x,&imu.gyro_i16.y,&imu.gyro_i16.z);
		MPU6500_GetMotion6(&imu.acc_i16.z,&imu.acc_i16.x,&imu.acc_i16.y,&imu.gyro_i16.z,&imu.gyro_i16.x,&imu.gyro_i16.y);
		
		imu.acc_i16.x = -imu.acc_i16.x;
		imu.acc_i16.y = imu.acc_i16.y;
		imu.acc_i16.z = imu.acc_i16.z;
		imu.gyro_i16.x = imu.gyro_i16.x;
		imu.gyro_i16.y = -imu.gyro_i16.y;
		imu.gyro_i16.z = -imu.gyro_i16.z;
		
		if(!(configs.imu.iscalibed&0x0100) || !(configs.imu.iscalibed&0x0001))
		{
			if(!(configs.imu.iscalibed&0x0100))
			{
				imuAddBiasValue(&imu.gyroBias, &imu.gyro_i16);

				imuFindBiasValue(&imu.gyroBias);
				
				if(configs.imu.iscalibed&0x0100)
				{
					Save_System_Config();
					//ledseqRun(SYS_LED, seq_calibrated);
				}
			}
			
			if(!(configs.imu.iscalibed&0x0001))
			{			
				rt_kprintf("acc,%d,%d,%d.\r\n",imu.acc_i16.x,imu.acc_i16.y,imu.acc_i16.z);
				
				if(configs.imu.iscalibed&0x0001)
				{
					
					//ledseqRun(SYS_LED, seq_calibrated);
				}
			}
		}
		
		if(configs.imu.iscalibed&0x0100)
		{
			imu.gyro_i16.x = imu.gyro_i16.x - configs.imu.gyro_Bx;
			imu.gyro_i16.y = imu.gyro_i16.y - configs.imu.gyro_By;
			imu.gyro_i16.z = imu.gyro_i16.z - configs.imu.gyro_Bz;

			tmp1 = imu.gyro_i16.x;
			tmp2 = imu.gyro_i16.y;
			
			imu.gyro_i16.x = (configs.imu.A[0] * tmp1 + configs.imu.A[1] * tmp2 + configs.imu.A[2] * imu.gyro_i16.z)/10000;
			imu.gyro_i16.y = (configs.imu.A[3] * tmp1 + configs.imu.A[4] * tmp2 + configs.imu.A[5] * imu.gyro_i16.z)/10000;
			imu.gyro_i16.z = (configs.imu.A[6] * tmp1 + configs.imu.A[7] * tmp2 + configs.imu.A[8] * imu.gyro_i16.z)/10000;			
		}
		
		if(configs.imu.iscalibed&0x0001)
		{
			imu.acc_i16.x = imu.acc_i16.x - configs.imu.acc_Bx;
			imu.acc_i16.y = imu.acc_i16.y - configs.imu.acc_By;
			imu.acc_i16.z = imu.acc_i16.z - configs.imu.acc_Bz;
			
			tmp1 = imu.acc_i16.x;
			tmp2 = imu.acc_i16.y;
			
			imu.acc_i16.x = (configs.imu.A[0] * tmp1 + configs.imu.A[1] * tmp2 + configs.imu.A[2] * imu.acc_i16.z)/10000;
			imu.acc_i16.y = (configs.imu.A[3] * tmp1 + configs.imu.A[4] * tmp2 + configs.imu.A[5] * imu.acc_i16.z)/10000;
			imu.acc_i16.z = (configs.imu.A[6] * tmp1 + configs.imu.A[7] * tmp2 + configs.imu.A[8] * imu.acc_i16.z)/10000;
		}

		imu.acc_f32.x = imu.acc_i16.x * imu.acc_scale;
		imu.acc_f32.y = imu.acc_i16.y * imu.acc_scale;
		imu.acc_f32.z = imu.acc_i16.z * imu.acc_scale;
		
		imu.gyro_f32.x = imu.gyro_i16.x * imu.gyro_scale;
		imu.gyro_f32.y = imu.gyro_i16.y * imu.gyro_scale;
		imu.gyro_f32.z = imu.gyro_i16.z * imu.gyro_scale;
		
	  if(imu_flag && cptcs%2==0) rt_sem_release(&att_sem);
	}
}

struct rt_semaphore att_sem;
int app_mpu6500_init(void)
{

	MPU6500_Init();
	
//	imuBiasInit(&imu.accelBias);
	imuBiasInit(&imu.gyroBias);
	
	rt_sem_init(&att_sem, "att_sem", 0, 0);
	
	return 0;
}
