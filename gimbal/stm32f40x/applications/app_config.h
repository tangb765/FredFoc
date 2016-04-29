
#include <rtthread.h>

#define VirtAddSTA               ((rt_uint16_t)0x6666)

struct pid_struct_t
{
	rt_int16_t  AbandonError;
	rt_int16_t  hKp_little;
	rt_int16_t  Integral_ambit_A;
	rt_int16_t  Integral_ambit_B;
	rt_int16_t  hKp_Gain;
	rt_uint16_t hKp_Divisor;
	rt_int16_t  hKi_Gain;
	rt_uint16_t hKi_Divisor;  
	rt_int16_t  hKd_Gain;
	rt_uint16_t hKd_Divisor;
  rt_int16_t  hLower_Limit_Output;     //Lower Limit for Output limitation
  rt_int16_t  hUpper_Limit_Output;     //Lower Limit for Output limitation
  rt_int32_t  wLower_Limit_Integral;   //Lower Limit for Integral term limitation
  rt_int32_t  wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
  rt_int32_t  wIntegral;
  rt_int32_t  wPreviousError;
};


struct imu_t
{
	rt_int16_t  iscalibed;
	rt_int16_t  gyro_Bx;
	rt_int16_t  gyro_By;
	rt_int16_t  gyro_Bz;
	rt_int16_t  acc_Bx;
	rt_int16_t  acc_By;
	rt_int16_t  acc_Bz;
	rt_int16_t  A[9];
};


struct SYSTEM_CONFIG
{
	rt_uint16_t Isvalid;
	rt_int16_t  encoder_align_pos;
	rt_int16_t  level_mec_angle;
	
	struct imu_t imu; 
	
	struct pid_struct_t pos_pid;
	struct pid_struct_t spd_pid;
	struct pid_struct_t tqe_pid;
	struct pid_struct_t flux_pid;
  
};

enum SYSTEM_CONFIG_ADDR
{
	Isvalid_addr = VirtAddSTA, //102 0x66
	encoder_align_pos_addr,
	level_mec_angle_addr,
	
	imu_iscalibed_addr,  //220
	imu_gyro_Bx_addr,
	imu_gyro_By_addr,
	imu_gyro_Bz_addr,
	imu_acc_Bx_addr,
	imu_acc_By_addr,
	imu_acc_Bz_addr,
	imu_A1_addr,    //227
	imu_A2_addr,
	imu_A3_addr,
	imu_A4_addr,
	imu_A5_addr,
	imu_A6_addr,	
	imu_A7_addr,
	imu_A8_addr,
	imu_A9_addr,	 //235
	
	
	pos_AbandonError_addr,
	pos_hKp_little_addr,
	pos_Integral_ambit_A_addr,
	pos_Integral_ambit_B_addr,
  pos_hKp_Gain_addr,  //105
  pos_hKp_Divisor_addr,
  pos_hKi_Gain_addr,
  pos_hKi_Divisor_addr,  
  pos_hKd_Gain_addr,
  pos_hKd_Divisor_addr,
  pos_hLower_Limit_Output_addr,
  pos_hUpper_Limit_Output_addr,
  pos_wLower_Limit_Integral_addr_up,
	pos_wLower_Limit_Integral_addr_down,
	pos_wUpper_Limit_Integral_addr_up,
  pos_wUpper_Limit_Integral_addr_down,
  pos_wIntegral_addr_up,
  pos_wIntegral_addr_down,
  pos_wPreviousError_addr_up,
	pos_wPreviousError_addr_down,	
	

	spd_AbandonError_addr,
	spd_hKp_little_addr,
	spd_Integral_ambit_A_addr,
	spd_Integral_ambit_B_addr,
  spd_hKp_Gain_addr, //121
  spd_hKp_Divisor_addr,
  spd_hKi_Gain_addr,
  spd_hKi_Divisor_addr,  
  spd_hKd_Gain_addr,
  spd_hKd_Divisor_addr,
  spd_hLower_Limit_Output_addr,
  spd_hUpper_Limit_Output_addr,
  spd_wLower_Limit_Integral_addr_up,
	spd_wLower_Limit_Integral_addr_down,
	spd_wUpper_Limit_Integral_addr_up,
  spd_wUpper_Limit_Integral_addr_down,
  spd_wIntegral_addr_up,
  spd_wIntegral_addr_down,
  spd_wPreviousError_addr_up,
	spd_wPreviousError_addr_down,
	
	tqe_AbandonError_addr,
	tqe_hKp_little_addr,
	tqe_Integral_ambit_A_addr,
	tqe_Integral_ambit_B_addr,	
  tqe_hKp_Gain_addr, //137
  tqe_hKp_Divisor_addr,
  tqe_hKi_Gain_addr,
  tqe_hKi_Divisor_addr, 
  tqe_hKd_Gain_addr,
  tqe_hKd_Divisor_addr,
  tqe_hLower_Limit_Output_addr,
  tqe_hUpper_Limit_Output_addr,
  tqe_wLower_Limit_Integral_addr_up,
	tqe_wLower_Limit_Integral_addr_down,
	tqe_wUpper_Limit_Integral_addr_up,
  tqe_wUpper_Limit_Integral_addr_down,
  tqe_wIntegral_addr_up,
  tqe_wIntegral_addr_down,
  tqe_wPreviousError_addr_up,
	tqe_wPreviousError_addr_down,
	
	flux_AbandonError_addr,
	flux_hKp_little_addr,
	flux_Integral_ambit_A_addr,
	flux_Integral_ambit_B_addr,		
  flux_hKp_Gain_addr, //153
  flux_hKp_Divisor_addr,
  flux_hKi_Gain_addr,
  flux_hKi_Divisor_addr,  
  flux_hKd_Gain_addr,
  flux_hKd_Divisor_addr,
  flux_hLower_Limit_Output_addr,
  flux_hUpper_Limit_Output_addr,
  flux_wLower_Limit_Integral_addr_up,
	flux_wLower_Limit_Integral_addr_down,
	flux_wUpper_Limit_Integral_addr_up,
  flux_wUpper_Limit_Integral_addr_down,
  flux_wIntegral_addr_up,
  flux_wIntegral_addr_down,
  flux_wPreviousError_addr_up,
	flux_wPreviousError_addr_down,
	
	VirtAddEND  //236
};

/* Variables' number :max<PAGE_SIZE/2/2=256 byte*/
#define NumbOfVar               ((rt_uint16_t)(VirtAddEND-VirtAddSTA))

extern rt_uint16_t VirtAddVarTab[NumbOfVar];
extern struct SYSTEM_CONFIG configs;

void Init_Config(void);
void Init_Default_Config(void);
void Save_System_Config(void);
