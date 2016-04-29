
#include <rtthread.h>


#define POSITIVE (1)
#define NEGATIVE (-1)

//0xf*inversion bit*roll_mode bit*pitch_mode bit*yaw_mode bit* 
enum SYS_MODE
{
	LOCK_MODE = 0,
	YAW_FOLLOW_MODE,
	PITCH_FOLLOW_MODE,
	YAW_AND_PITCH_FOLLOW_MODE,
	ROLL_FOLLOW_MODE,
	YAW_AND_ROLL_FOLLOW_MODE,
	PITCH_AND_ROLL_FOLLOW_MODE,
	FOLLOW_MODE,
	INVERSION_LOCK_MODE,
	INVERSION_YAW_FOLLOW_MODE,
	INVERSION_PITCH_FOLLOW_MODE,
	INVERSION_YAW_AND_PITCH_FOLLOW_MODE,
	INVERSION_ROLL_FOLLOW_MODE,
	INVERSION_YAW_AND_ROLL_FOLLOW_MODE,
	INVERSION_PITCH_AND_ROLL_FOLLOW_MODE,
	INVERSION_FOLLOW_MODE,
	MODE_NUMS,
};

enum MOTOR_STATE
{
	NORMAL = 0,
	POS_DEBUG,
	SPD_DEBUG,
	ID_DEBUG,
	IQ_DEBUG,
};

enum OUTPUT_FLAG
{
	CLOSE = 0,
	IMU,
	MOTOR,
	TLE5012,
	MPU6500,
};


extern enum MOTOR_STATE motor_state;
extern rt_uint8_t console_flag;
extern enum OUTPUT_FLAG output_flag;

extern volatile rt_int16_t eulerActual;
extern volatile rt_int16_t eulerspeedActual;
extern enum SYS_MODE system_mode;
extern enum SYS_MODE system_mode_last;
extern rt_int8_t first_times;
extern rt_uint8_t imu_flag;

void imu_protocol(rt_uint8_t *buf,rt_size_t size);
void extern_cmd_protocol(rt_uint8_t *buf,rt_size_t size);
