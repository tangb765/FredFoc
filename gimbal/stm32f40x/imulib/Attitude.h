
#include <rtthread.h>
#include <stdint.h>

#define GRAVITY   9.7925f 
#define DEGTORAD  0.01748081f

#define IMU_NBR_OF_BIAS_SAMPLES  128

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)


#define GYRO_MIN_BIAS_TIMEOUT_MS    (1000)


typedef struct {
	rt_int16_t x;
	rt_int16_t y;
	rt_int16_t z;
} Axis3i16;

typedef struct {
	rt_int32_t x;
	rt_int32_t y;
	rt_int32_t z;
} Axis3i32;

typedef struct {
	float x;
	float y;
	float z;
} Axis3f;


typedef struct
{
  rt_bool_t  isBiasValueFound;
  rt_bool_t  isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;


typedef struct
{
	uint64_t    timestamp;
	Axis3i16    acc_i16;
	BiasObj     accelBias;
	Axis3f      acc_f32;
	float       acc_scale;
	Axis3i16    gyro_i16;
	BiasObj     gyroBias;
	Axis3f      gyro_f32;
	float       gyro_scale;
	rt_bool_t   valid; 
}imu_s;


extern imu_s  imu;
extern struct rt_semaphore att_sem;

void attitude_thread_entry(void* parameter);
int app_attitude_init(void);
