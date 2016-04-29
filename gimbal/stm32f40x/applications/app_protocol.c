#include "board.h"
#include "app_protocol.h"
#include <stdlib.h>
#include "TLE5012.h"
#include "hrt.h"
#include "MC_Globals.h"
#include "stm32f10x_encoder.h"
#include "MC_const.h"
#include "MC_PID_regulators.h"


volatile rt_int16_t eulerActual = 0;
volatile rt_int16_t eulerspeedActual = 0;
enum SYS_MODE system_mode = YAW_FOLLOW_MODE;
enum SYS_MODE system_mode_last = YAW_FOLLOW_MODE;

uint64_t varianceSampleTime;
enum MOTOR_STATE motor_state = NORMAL;
enum MOTOR_STATE motor_state_Last = NORMAL;
rt_uint8_t console_flag = CONSOLE_ALL | CONSOLE_A;
enum OUTPUT_FLAG output_flag = CLOSE;
rt_uint8_t imu_flag = 1;
extern volatile rt_int16_t hSpeed_Reference;

rt_int8_t first_times = -50;
static uint64_t last_timestamp = 0;
static uint64_t cur_timestamp = 0;

void imu_protocol(rt_uint8_t *buf,rt_size_t size)
{
	if(rt_strstr((void*)buf,"#E,") != RT_NULL)
	{
		rt_size_t i;
		rt_size_t pos_s = 0;
		rt_size_t pos_p = 0;
		rt_size_t pos_q = 0;
		rt_size_t pos_e = 0;
		rt_size_t pos_f = 0;
		rt_size_t pos_g = 0;
		rt_size_t pos_h = 0;
		rt_uint8_t *buf_tmp;
		rt_uint8_t *buf_tmp1;
		
		for(i=0;i<size;i++)
		{
			if(pos_s == 0 && buf[i]==',')
			{
				pos_s = i;
				continue;
			}
			
			if(pos_p == 0 && pos_s != 0 && buf[i]==',')
			{
				pos_p = i;
				continue;
			}
			
			if(pos_q == 0 && pos_p != 0 && buf[i]==',')
			{
				pos_q = i;
				continue;
			}
			
			if(pos_e == 0 && pos_q != 0 && buf[i]==',')
			{
				pos_e = i;
				continue;
			}
			
			if(pos_f == 0 && pos_e != 0 && buf[i]==',')
			{
				pos_f = i;
				continue;
			}
			
			if(pos_g == 0 && pos_f != 0 && buf[i]==',')
			{
				pos_g = i;
				continue;
			}
			
			if(pos_h == 0 && pos_g != 0 && buf[i]=='$')
			{
				pos_h = i;
				continue;
			}
		}
		
		if(pos_s != 0 && pos_p != 0 && pos_q != 0 && pos_e != 0)
		{
			cur_timestamp = hrt_absolute_time();
			
			if(first_times<-49)
			{
				if(first_times == -50) 
				{
					State = INIT;
				first_times++;
				}
				last_timestamp = cur_timestamp;
//				return;
			}
			
//			if(cur_timestamp - last_timestamp > HRT_MS_COUNT*10)
//			{
//				first_times++;
//				if(first_times == 0)
//				{
//					State = STOP;
//					rt_kprintf("%c,att timeout.\r\n",CONSOLE_O-1+'A');
//				}
////				first_times = 51;
//				return;
//			}
//			else
//			{
				last_timestamp = cur_timestamp;
//			}
			
			#ifdef FeiYu_Board_A
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_p- pos_s));
				rt_memcpy(buf_tmp,&buf[pos_s+1],(pos_p- pos_s - 1));
				buf_tmp[(pos_p- pos_s - 1)] = '\0';
				eulerActual = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
			
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_f- pos_e));
				rt_memcpy(buf_tmp1,&buf[pos_e+1],(pos_f- pos_e - 1));
				buf_tmp1[(pos_f- pos_e - 1)] = '\0';
				eulerspeedActual = atoi((void*)buf_tmp1);
				rt_free(buf_tmp1);
			#endif
			
			#ifdef FeiYu_Board_B
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_q - pos_p));
				rt_memcpy(buf_tmp,&buf[pos_p+1],(pos_q - pos_p - 1));
				buf_tmp[(pos_q - pos_p - 1)] = '\0';
				eulerActual = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
			
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_g- pos_f));
				rt_memcpy(buf_tmp1,&buf[pos_f+1],(pos_g- pos_f - 1));
				buf_tmp1[(pos_g- pos_f - 1)] = '\0';
				eulerspeedActual = atoi((void*)buf_tmp1);
				rt_free(buf_tmp1);
			#endif

			#ifdef FeiYu_Board_C
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_e - pos_q));
				rt_memcpy(buf_tmp,&buf[pos_q+1],(pos_e - pos_q - 1));
				buf_tmp[(pos_e - pos_q - 1)] = '\0';
				eulerActual = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
				
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_h- pos_g));
				rt_memcpy(buf_tmp1,&buf[pos_g+1],(pos_h- pos_g - 1));
				buf_tmp1[(pos_h- pos_g - 1)] = '\0';
				eulerspeedActual = atoi((void*)buf_tmp1);
				rt_free(buf_tmp1);
			#endif	
		}
		return;
	}
}


rt_uint8_t eximu_protocol(rt_uint8_t *buf,rt_size_t size)
{
	if(rt_strstr((void*)buf,"#E,") != RT_NULL)
	{
		rt_size_t i;
		rt_size_t pos_s = 0;
		rt_size_t pos_p = 0;
		rt_size_t pos_q = 0;
		rt_size_t pos_e = 0;
		rt_size_t pos_f = 0;
		rt_size_t pos_g = 0;
		rt_size_t pos_h = 0;
		rt_uint8_t *buf_tmp;
		rt_uint8_t *buf_tmp1;
		
		for(i=0;i<size;i++)
		{
			if(pos_s == 0 && buf[i]==',')
			{
				pos_s = i;
				continue;
			}
			
			if(pos_p == 0 && pos_s != 0 && buf[i]==',')
			{
				pos_p = i;
				continue;
			}
			
			if(pos_q == 0 && pos_p != 0 && buf[i]==',')
			{
				pos_q = i;
				continue;
			}
			
			if(pos_e == 0 && pos_q != 0 && buf[i]==',')
			{
				pos_e = i;
				continue;
			}
			
			if(pos_f == 0 && pos_e != 0 && buf[i]==',')
			{
				pos_f = i;
				continue;
			}
			
			if(pos_g == 0 && pos_f != 0 && buf[i]==',')
			{
				pos_g = i;
				continue;
			}
			
			if(pos_h == 0 && pos_g != 0 && buf[i]=='@')
			{
				pos_h = i;
				continue;
			}
		}
		
		if(pos_s != 0 && pos_p != 0 && pos_q != 0 && pos_e != 0)
		{
			#ifdef FeiYu_Board_A
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_p- pos_s));
				rt_memcpy(buf_tmp,&buf[pos_s+1],(pos_p- pos_s - 1));
				buf_tmp[(pos_p- pos_s - 1)] = '\0';
				hPosition_reference = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
			
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_f- pos_e));
				rt_memcpy(buf_tmp1,&buf[pos_e+1],(pos_f- pos_e - 1));
				buf_tmp1[(pos_f- pos_e - 1)] = '\0';		
				hSpeed_Reference = atoi((void*)buf_tmp1);	
				rt_free(buf_tmp1);
			#endif
			
			#ifdef FeiYu_Board_B
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_q - pos_p));
				rt_memcpy(buf_tmp,&buf[pos_p+1],(pos_q - pos_p - 1));
				buf_tmp[(pos_q - pos_p - 1)] = '\0';
				hPosition_reference = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
			
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_g- pos_f));
				rt_memcpy(buf_tmp1,&buf[pos_f+1],(pos_g- pos_f - 1));
				buf_tmp1[(pos_g- pos_f - 1)] = '\0';
				hSpeed_Reference = atoi((void*)buf_tmp1);
				rt_free(buf_tmp1);
			#endif

			#ifdef FeiYu_Board_C
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_e - pos_q));
				rt_memcpy(buf_tmp,&buf[pos_q+1],(pos_e - pos_q - 1));
				buf_tmp[(pos_e - pos_q - 1)] = '\0';
				hPosition_reference = atoi((void*)buf_tmp);
				rt_free(buf_tmp);
				
				buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_h- pos_g));
				rt_memcpy(buf_tmp1,&buf[pos_g+1],(pos_h- pos_g - 1));
				buf_tmp1[(pos_h- pos_g - 1)] = '\0';
				hSpeed_Reference = atoi((void*)buf_tmp1);
				rt_free(buf_tmp1);
			#endif	
		}	
		return 1;
	}
	
	return 0;
}

void extern_cmd_protocol(rt_uint8_t *buf,rt_size_t size)
{
	if(rt_strstr((void*)buf,"#mode,") != RT_NULL)
	{
		rt_uint8_t *p,*pl;
		rt_uint8_t *buf_tmp;
		rt_int8_t buf_tmp1;
		pl=&buf[6];
		
		if((p=(void *)rt_strstr((void *)pl,"@"))!=RT_NULL)
		{
			buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(p- pl+1));
			rt_memcpy(buf_tmp,pl,(p- pl));
			buf_tmp[(p- pl)] = '\0';
			buf_tmp1 = atoi((void*)buf_tmp);

			if((buf_tmp1<MODE_NUMS) && (buf_tmp1>=LOCK_MODE))
			{
				system_mode = buf_tmp1;
				
				if(system_mode!=system_mode_last)
				{
					hPosition_reference = 0;
					system_mode_last = system_mode;
				}
			}
			
			rt_free(buf_tmp);
		}
	}
	
	if ((console_flag&0xf0) == CONSOLE_ALL || (console_flag&0x0f) == CONSOLE_O)
	{
		if (eximu_protocol(buf,size) == 1)
		{
			return;
		}
		
		if(rt_strstr((void*)buf,"#motor,") != RT_NULL)
		{
			rt_uint8_t tmp = 0;
			
			if(rt_strstr((void*)(&buf[7]),"run@") != RT_NULL)               tmp = tmp + 1;
			else if(rt_strstr((void*)(&buf[7]),"stop@") != RT_NULL)         tmp = tmp + 2;
			else if(rt_strstr((void*)(&buf[7]),"calilevel@") != RT_NULL)    tmp = tmp + 3;
			else if(rt_strstr((void*)(&buf[7]),"align@") != RT_NULL)        tmp = tmp + 4;
		  else if(rt_strstr((void*)(&buf[7]),"test,speed@") != RT_NULL)   tmp = tmp + 5;
			else if(rt_strstr((void*)(&buf[7]),"test,idcurrent@") != RT_NULL) tmp = tmp + 6;
			else if(rt_strstr((void*)(&buf[7]),"test,iqcurrent@") != RT_NULL) tmp = tmp + 7;
//			else if(rt_strstr((void*)(&buf[7]),"test,position@") != RT_NULL)tmp = tmp + 8;
			else if(rt_strstr((void*)(&buf[7]),"test,close@") != RT_NULL)   tmp = tmp + 8;
			else rt_kprintf("error motor command.\r\n");
			
			if(tmp>=3 && (console_flag&0x0f) != CONSOLE_O) return;
			
			switch(tmp)
			{		
				case 1:
					if(State != RUN) State = INIT;
					break;
				
				case 2:
					State = STOP;
					break;

				case 3:
					configs.level_mec_angle = 0;
					configs.level_mec_angle = Get_Mec_Angle_Value();
				  configs.Isvalid = configs.Isvalid | 0x0080;
					break;			
				
				case 4:
					State = INIT;
					wGlobal_Flags = FIRST_START | SPEED_CONTROL;
					break;	
				
				case 5:
					motor_state = SPD_DEBUG;
					wGlobal_Flags = SPEED_CONTROL;
					break;
					
				case 6:
					motor_state = ID_DEBUG;
					wGlobal_Flags = TORQUE_CONTROL;
					break;		
					
				case 7:
					motor_state = IQ_DEBUG;
					wGlobal_Flags = TORQUE_CONTROL;
					break;
					
//				case 8:
//					motor_state = POS_DEBUG;
//					wGlobal_Flags = SPEED_CONTROL;
//					break;
				
				case 8:
					motor_state = NORMAL;
					wGlobal_Flags = SPEED_CONTROL;
					break;
		
				
				default:
					break;
			}
			
			if(motor_state_Last != motor_state)
			{
				State = STOP;
				motor_state_Last = motor_state;
			}
			
			return;
		}
	}
	
	if ((console_flag&0x0f) == CONSOLE_O)
	{
		if(rt_strstr((void*)buf,"#save,system,configs@") != RT_NULL)
		{
			Save_System_Config();
			return;
		}
		
		if(rt_strstr((void*)buf,"#test,") != RT_NULL)
		{
			if(rt_strstr((void*)(&buf[6]),"close@") != RT_NULL)            output_flag = CLOSE;
			else if(rt_strstr((void*)(&buf[6]),"tle5012@") != RT_NULL)     output_flag = TLE5012;
			#ifdef FeiYu_Board_A
			else if(rt_strstr((void*)(&buf[6]),"mpu6500@") != RT_NULL)     output_flag = MPU6500;
			#endif
			else if(rt_strstr((void*)(&buf[6]),"motor@") != RT_NULL)       output_flag = MOTOR;	
			else rt_kprintf("error test command.\r\n");
			
			return;
		}	

		if(rt_strstr((void*)buf,"#get,") != RT_NULL)
		{
			output_flag = CLOSE;
			if(rt_strstr((void*)(&buf[5]),"align@") != RT_NULL)          
				rt_kprintf("encoder_align_pos=%d\r\n",configs.encoder_align_pos);
			else if(rt_strstr((void*)(&buf[5]),"calilevel@") != RT_NULL)
				rt_kprintf("level_mec_angle=%d\r\n",configs.level_mec_angle);
			else if(rt_strstr((void*)(&buf[5]),"pid,") != RT_NULL)    
			{
				rt_uint8_t getcnt = 0;
				rt_int16_t hGain = 0;
				rt_uint16_t hDivisor = 0;
				if(rt_strstr((void*)(&buf[9]),"pk@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.pos_pid.hKp_Gain;
					hDivisor = configs.pos_pid.hKp_Divisor;
					rt_kprintf("pk,");
				}
				else if(rt_strstr((void*)(&buf[9]),"pi@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.pos_pid.hKi_Gain;
					hDivisor = configs.pos_pid.hKi_Divisor;
					rt_kprintf("pi,");
				}
				else if(rt_strstr((void*)(&buf[9]),"pd@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.pos_pid.hKd_Gain;
					hDivisor = configs.pos_pid.hKd_Divisor;
					rt_kprintf("pd,");
				}
				else if(rt_strstr((void*)(&buf[9]),"sk@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.spd_pid.hKp_Gain;
					hDivisor = configs.spd_pid.hKp_Divisor;
					rt_kprintf("sk,");
				}
				else if(rt_strstr((void*)(&buf[9]),"si@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.spd_pid.hKi_Gain;
					hDivisor = configs.spd_pid.hKi_Divisor;
					rt_kprintf("si,");
				}
				else if(rt_strstr((void*)(&buf[9]),"sd@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.spd_pid.hKd_Gain;
					hDivisor = configs.spd_pid.hKd_Divisor;
					rt_kprintf("sd,");
				}
				else if(rt_strstr((void*)(&buf[9]),"tk@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.tqe_pid.hKp_Gain;
					hDivisor = configs.tqe_pid.hKp_Divisor;
					rt_kprintf("tk,");
				}
				else if(rt_strstr((void*)(&buf[9]),"ti@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.tqe_pid.hKi_Gain;
					hDivisor = configs.tqe_pid.hKi_Divisor;
					rt_kprintf("ti,");
				}
				else if(rt_strstr((void*)(&buf[9]),"td@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.tqe_pid.hKd_Gain;
					hDivisor = configs.tqe_pid.hKd_Divisor;
					rt_kprintf("td,");
				}
				else if(rt_strstr((void*)(&buf[9]),"fk@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.flux_pid.hKp_Gain;
					hDivisor = configs.flux_pid.hKp_Divisor;
					rt_kprintf("fk,");
				}
				else if(rt_strstr((void*)(&buf[9]),"fi@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.flux_pid.hKi_Gain;
					hDivisor = configs.flux_pid.hKi_Divisor;
					rt_kprintf("fi,");
				}
				else if(rt_strstr((void*)(&buf[9]),"fd@") != RT_NULL)  
				{
					getcnt = 1;
					hGain = configs.flux_pid.hKd_Gain;
					hDivisor = configs.flux_pid.hKd_Divisor;
					rt_kprintf("fd,");
				}
				else
				{
					rt_kprintf("error,get,command.\r\n");
				}
				
				if(getcnt)
					rt_kprintf("hGain = %d,hDivisor = %d.\r\n",hGain,hDivisor);
			}
			else
			{
				rt_kprintf("error,get,command.\r\n");
			}
			
			return;
		}			

		
		if(rt_strstr((void*)buf,"#pid,") != RT_NULL)
		{
			rt_uint8_t tmp = 0;
			
			if(rt_strstr((void*)(&buf[5]),"tk,") != RT_NULL)           tmp = tmp + 1;
			else if(rt_strstr((void*)(&buf[5]),"ti,") != RT_NULL)      tmp = tmp + 2;
			else if(rt_strstr((void*)(&buf[5]),"td,") != RT_NULL)      tmp = tmp + 3;
			else if(rt_strstr((void*)(&buf[5]),"fk,") != RT_NULL)      tmp = tmp + 4;
			else if(rt_strstr((void*)(&buf[5]),"fi,") != RT_NULL)      tmp = tmp + 5;
			else if(rt_strstr((void*)(&buf[5]),"fd,") != RT_NULL)      tmp = tmp + 6;
			else if(rt_strstr((void*)(&buf[5]),"sk,") != RT_NULL)      tmp = tmp + 7;
			else if(rt_strstr((void*)(&buf[5]),"si,") != RT_NULL)      tmp = tmp + 8;
			else if(rt_strstr((void*)(&buf[5]),"sd,") != RT_NULL)      tmp = tmp + 9;
			else if(rt_strstr((void*)(&buf[5]),"pk,") != RT_NULL)      tmp = tmp + 10;
			else if(rt_strstr((void*)(&buf[5]),"pi,") != RT_NULL)      tmp = tmp + 11;
			else if(rt_strstr((void*)(&buf[5]),"pd,") != RT_NULL)      tmp = tmp + 12;
			else rt_kprintf("error pid set command.\r\n");

			
			if(tmp>=1 && tmp<=12)
			{
				rt_int16_t hGain = 0;
				rt_uint16_t hDivisor = 0;
				
				rt_size_t i;
				rt_size_t pos_s = 0;
				rt_size_t pos_p = 0;
				rt_size_t pos_e = 0;
				rt_uint8_t *buf_tmp;
				rt_uint8_t *buf_tmp1;
				
				for(i=7;i<size;i++)
				{
					if(pos_s == 0 && buf[i]==',')
					{
						pos_s = i;
						continue;
					}
					
					if(pos_p == 0 && pos_s != 0 && buf[i]==',')
					{
						pos_p = i;
						continue;
					}
					
					if(pos_e == 0 && pos_p != 0 && buf[i]=='@')
					{
						pos_e = i;
						continue;
					}
				}
				
				if(pos_s != 0 && pos_p != 0 && pos_e != 0)
				{
					buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(pos_p- pos_s));
					rt_memcpy(buf_tmp,&buf[pos_s+1],(pos_p- pos_s - 1));
					buf_tmp[(pos_p- pos_s - 1)] = '\0';
					hGain = atoi((void*)buf_tmp);
					rt_free(buf_tmp);
					
					buf_tmp1 = rt_malloc(sizeof(rt_uint8_t)*(pos_e- pos_p - 1));
					rt_memcpy(buf_tmp1,&buf[pos_p+1],(pos_e- pos_p - 1));
					buf_tmp1[(pos_e- pos_p - 1)] = '\0';
					hDivisor = atoi((void*)buf_tmp1);
					rt_free(buf_tmp1);
					
					
					
					switch(tmp)
					{
						case 1:
								rt_kprintf("tk,");
								PID_Torque_Kp_update(hGain,hDivisor);
							break;
						
						case 2:
								rt_kprintf("ti,");
								PID_Torque_Ki_update(hGain,hDivisor);
							break;
						
						case 3:
								rt_kprintf("td,");
								PID_Torque_Kd_update(hGain,hDivisor);
							break;
						
						case 4:
								rt_kprintf("fk,");
								PID_Flux_Kp_update(hGain,hDivisor);
							break;
						
						case 5:
								rt_kprintf("fi,");
								PID_Flux_Ki_update(hGain,hDivisor);
							break;
						
						case 6:
								rt_kprintf("fd,");
								PID_Flux_Kd_update(hGain,hDivisor);
							break;
						
						case 7:
								rt_kprintf("sk,");
								PID_Speed_Kp_update(hGain,hDivisor);
							break;
						
						case 8:
								rt_kprintf("si,");
								PID_Speed_Ki_update(hGain,hDivisor);
							break;
						
						case 9:
								rt_kprintf("sd,");
								PID_Speed_Kd_update(hGain,hDivisor);
							break;
						
						case 10:
								rt_kprintf("pk,");
								PID_Position_Kp_update(hGain,hDivisor);
							break;
						
						case 11:
								rt_kprintf("pi,");
								PID_Position_Ki_update(hGain,hDivisor);
							break;
						
						case 12:
								rt_kprintf("pd,");
								PID_Position_Kd_update(hGain,hDivisor);
							break;	
						
						default:
							rt_kprintf("error,");
							break;
					}
					
					rt_kprintf("hGain = %d,hDivisor = %d.\r\n",hGain,hDivisor);
				}
			}
			
			return;
		}
	}
	
	
	if(rt_strstr((void*)buf,"#console,") != RT_NULL)
	{
		rt_uint8_t tmp = 0;
		
		if(rt_strstr((void*)(&buf[9]),"a@") != RT_NULL)           tmp = tmp + 1;
		else if(rt_strstr((void*)(&buf[9]),"b@") != RT_NULL)      tmp = tmp + 2;
		else if(rt_strstr((void*)(&buf[9]),"c@") != RT_NULL)      tmp = tmp + 3;
		else if(rt_strstr((void*)(&buf[9]),"null@") != RT_NULL)   tmp = tmp + 4;
		else if(rt_strstr((void*)(&buf[9]),"all@") != RT_NULL)    tmp = tmp + 5;
		
		switch(tmp)
		{
			case 1:
				if((console_flag & 0xf0) == CONSOLE_ALL)
					console_flag = CONSOLE_ALL | CONSOLE_A;
				else
					console_flag = CONSOLE_A;
				break;
			
			case 2:
				if((console_flag & 0xf0) == CONSOLE_ALL)
					console_flag = CONSOLE_ALL | CONSOLE_B;
				else
					console_flag = CONSOLE_B;
				break;

			case 3:
				if((console_flag & 0xf0) == CONSOLE_ALL)
					console_flag = CONSOLE_ALL | CONSOLE_C;
				else
					console_flag = CONSOLE_C;
				break;			
			
			case 4:
				console_flag = CONSOLE_NULL;
				break;
			
			case 5:
				if((console_flag & 0xf0) == CONSOLE_ALL)
					console_flag = console_flag & 0x0f;
				else
					console_flag = console_flag | CONSOLE_ALL;
				break;
			
			default:
				rt_kprintf("error console command.\r\n");
				break;
		}
		
//		if((console_flag&0x0f) != CONSOLE_O)
//		{
//			output_flag = CLOSE;
//			State = STOP;
//			hPosition_reference = 0;
//			hSpeed_Reference = 0;		
//		}
		
		
		return;
	}

#ifdef FeiYu_Board_A	
	if(rt_strstr((void*)buf,"#imu,") != RT_NULL)
	{
		if(rt_strstr((void*)(&buf[5]),"open@") != RT_NULL)
		{
			imu_flag = 1;
			State = STOP;
			app_attitude_init();
			first_times = -50;
//			output_flag = IMU;
		}
		else if(rt_strstr((void*)(&buf[5]),"close@") != RT_NULL)
		{
			imu_flag = 0;
			State = STOP;
			first_times = -50;
//      output_flag = CLOSE;
		}
		else if(rt_strstr((void*)(&buf[5]),"gyro,calib@") != RT_NULL)
		{
			State = STOP;
      configs.imu.iscalibed &= (~0x0100);
			varianceSampleTime = hrt_absolute_ms();
		}
		else if(rt_strstr((void*)(&buf[5]),"acc,calib@") != RT_NULL)
		{
			State = STOP;
			configs.imu.iscalibed &= (~0x0001);
			varianceSampleTime = hrt_absolute_ms();
		}
		else if(rt_strstr((void*)(&buf[5]),"acc,calib,") != RT_NULL)
		{
			rt_uint8_t *p,*pl;
			rt_uint8_t *buf_tmp;
			rt_int16_t buf_tmp1[12];
			rt_uint8_t  buf_cnt = 0;
			pl=&buf[15];
			while((p=(void *)rt_strstr((void *)pl,","))!=RT_NULL)
			{
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(p- pl+1));
				rt_memcpy(buf_tmp,pl,(p- pl));
				buf_tmp[(p- pl)] = '\0';
				pl = p+1;
				
				if(buf_cnt<11)
				{
					buf_tmp1[buf_cnt] = atoi((void*)buf_tmp);			
				}
				buf_cnt++;
				rt_free(buf_tmp);
			}
			
			if(buf_cnt==11 && (p=(void *)rt_strstr((void *)pl,"@"))!=RT_NULL)
			{
				buf_tmp = rt_malloc(sizeof(rt_uint8_t)*(p- pl+1));
				rt_memcpy(buf_tmp,pl,(p- pl));
				buf_tmp[(p- pl)] = '\0';
				buf_tmp1[buf_cnt] = atoi((void*)buf_tmp);	
				rt_memcpy(&configs.imu.acc_Bx,buf_tmp1,12*sizeof(rt_int16_t));
				rt_free(buf_tmp);
			}
		}
		else if(rt_strstr((void*)(&buf[5]),"get,") != RT_NULL)
		{
			output_flag = CLOSE;
			if(rt_strstr((void*)(&buf[9]),"state@") != RT_NULL)
			{
				rt_kprintf("imu state:gyro calb %s,acc calib %s\r\n",configs.imu.iscalibed&0x0100 ? "YES" : "NO",configs.imu.iscalibed&0x0001 ? "YES" : "NO");
			}
			else if(rt_strstr((void*)(&buf[9]),"gyro,bias@") != RT_NULL)
			{
				rt_kprintf("gyro_Bias=%d,%d,%d\r\n",configs.imu.gyro_Bx,configs.imu.gyro_By,configs.imu.gyro_Bz);
			}			
			else if(rt_strstr((void*)(&buf[9]),"acc,bias@") != RT_NULL)
			{
				rt_kprintf("acc_Bias=%d,%d,%d\r\n",configs.imu.acc_Bx,configs.imu.acc_By,configs.imu.acc_Bz);
			}
			else if(rt_strstr((void*)(&buf[9]),"acc,martixA@") != RT_NULL)
			{
				rt_kprintf("acc_A=%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",configs.imu.A[0],configs.imu.A[1],configs.imu.A[2],
									configs.imu.A[3],configs.imu.A[4],configs.imu.A[5],configs.imu.A[6],configs.imu.A[7],configs.imu.A[8]);
			}
			else
			{
				rt_kprintf("error imu get command.\r\n");
			}
		}
	  else
		{
			rt_kprintf("error imu command.\r\n");
		}
		
		
		return;
	}
#endif	
}

