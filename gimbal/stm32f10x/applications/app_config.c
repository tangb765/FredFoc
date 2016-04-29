

#include "board.h"
#include "virtual_eeprom.h"
#include "app_config.h"



extern void Init_Config_PID(void);
extern void Init_PID(void);

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar];

struct SYSTEM_CONFIG configs;
//no check
void Save_System_Config(void)
{
	rt_uint16_t viraddr = 0;
	
	for (viraddr = VirtAddSTA; viraddr < VirtAddEND; viraddr++)
	{
		EE_WriteVariable(viraddr, *(&configs.Isvalid + (viraddr-VirtAddSTA)));
	}
}

void Init_Default_Config(void)
{
	configs.Isvalid                       = 0x8081;
	configs.encoder_align_pos             = Encoder_Align_Pos;
	configs.level_mec_angle               = Level_Mec_Angle;
	
	Init_Config_PID();
	
	configs.imu.iscalibed                  = 0x0101;
	configs.imu.acc_Bx                     = 0;
	configs.imu.acc_By                     = 0;
	configs.imu.acc_Bz                     = 0;
	configs.imu.gyro_Bx                    = 0;
	configs.imu.gyro_By                    = 0;
	configs.imu.gyro_Bz                    = 0;
	configs.imu.A[0]                       = 1;                       
	configs.imu.A[1]                       = 0;
	configs.imu.A[2]                       = 0;
	configs.imu.A[3]                       = 0;
	configs.imu.A[4]                       = 1;
	configs.imu.A[5]                       = 0;
	configs.imu.A[6]                       = 0;
	configs.imu.A[7]                       = 0;
	configs.imu.A[8]                       = 1;
	
	Save_System_Config();
}


void Init_Config(void)
{
	rt_uint16_t viraddr  = 0;
	rt_uint16_t VarValue = 0;

	EE_Init();
	
	EE_ReadVariable(VirtAddSTA, &VarValue);
	
  /* Check if first running */
  if (VarValue == 0)
  {
		Init_Default_Config();
  }
	
	for (viraddr = VirtAddSTA; viraddr < VirtAddEND; viraddr++)
	{		
		EE_ReadVariable(viraddr,&configs.Isvalid + (viraddr-VirtAddSTA));
	}
	
	Init_PID();

	
}

