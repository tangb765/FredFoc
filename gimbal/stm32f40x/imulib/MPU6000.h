#ifndef __MPU6500_H_
#define __MPU6500_H_

#include <stdint.h>
#include <rtthread.h>
 
 
 
/*============================================================================*/
/*                              Macro Definition                              */
/*============================================================================*/
#define MPU6500_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6500_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6500_DEFAULT_ADDRESS     MPU6500_ADDRESS_AD0_LOW


#define MPU6500_RA_SELF_TEST_X_GYRO       0x00 
#define MPU6500_RA_SELF_TEST_Y_GYRO       0x01 
#define MPU6500_RA_SELF_TEST_Z_GYRO       0x02 

#define MPU6500_RA_SELF_TEST_X_ACCEL      0x0D
#define MPU6500_RA_SELF_TEST_Y_ACCEL      0x0E
#define MPU6500_RA_SELF_TEST_Z_ACCEL      0x0F

#define MPU6500_RA_XG_OFFS_USRH           0x13 //[15:0] XG_OFFS_USR
#define MPU6500_RA_XG_OFFS_USRL           0x14
#define MPU6500_RA_YG_OFFS_USRH           0x15 //[15:0] YG_OFFS_USR
#define MPU6500_RA_YG_OFFS_USRL           0x16
#define MPU6500_RA_ZG_OFFS_USRH           0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_RA_ZG_OFFS_USRL           0x18

#define MPU6500_RA_SMPLRT_DIV             0x19
#define MPU6500_RA_CONFIG                 0x1A
#define MPU6500_RA_GYRO_CONFIG            0x1B
#define MPU6500_RA_ACCEL_CONFIG           0x1C
#define MPU6500_RA_ACCEL_CONFIG2          0x1D
#define MPU6500_RA_LP_ACCEL_ODR           0x1E

#define MPU6500_RA_WOM_THR                0x1F
#define MPU6500_RA_FIFO_EN                0x23

#define MPU6500_RA_I2C_MST_CTRL           0x24
#define MPU6500_RA_I2C_SLV0_ADDR          0x25
#define MPU6500_RA_I2C_SLV0_REG           0x26
#define MPU6500_RA_I2C_SLV0_CTRL          0x27
#define MPU6500_RA_I2C_SLV1_ADDR          0x28
#define MPU6500_RA_I2C_SLV1_REG           0x29
#define MPU6500_RA_I2C_SLV1_CTRL          0x2A
#define MPU6500_RA_I2C_SLV2_ADDR          0x2B
#define MPU6500_RA_I2C_SLV2_REG           0x2C
#define MPU6500_RA_I2C_SLV2_CTRL          0x2D
#define MPU6500_RA_I2C_SLV3_ADDR          0x2E
#define MPU6500_RA_I2C_SLV3_REG           0x2F
#define MPU6500_RA_I2C_SLV3_CTRL          0x30
#define MPU6500_RA_I2C_SLV4_ADDR          0x31
#define MPU6500_RA_I2C_SLV4_REG           0x32
#define MPU6500_RA_I2C_SLV4_DO            0x33
#define MPU6500_RA_I2C_SLV4_CTRL          0x34
#define MPU6500_RA_I2C_SLV4_DI            0x35
#define MPU6500_RA_I2C_MST_STATUS         0x36

#define MPU6500_RA_INT_PIN_CFG            0x37
#define MPU6500_RA_INT_ENABLE             0x38
#define MPU6500_RA_INT_STATUS             0x3A

#define MPU6500_RA_ACCEL_XOUT_H           0x3B
#define MPU6500_RA_ACCEL_XOUT_L           0x3C
#define MPU6500_RA_ACCEL_YOUT_H           0x3D
#define MPU6500_RA_ACCEL_YOUT_L           0x3E
#define MPU6500_RA_ACCEL_ZOUT_H           0x3F
#define MPU6500_RA_ACCEL_ZOUT_L           0x40
#define MPU6500_RA_TEMP_OUT_H             0x41
#define MPU6500_RA_TEMP_OUT_L             0x42
#define MPU6500_RA_GYRO_XOUT_H            0x43
#define MPU6500_RA_GYRO_XOUT_L            0x44
#define MPU6500_RA_GYRO_YOUT_H            0x45
#define MPU6500_RA_GYRO_YOUT_L            0x46
#define MPU6500_RA_GYRO_ZOUT_H            0x47
#define MPU6500_RA_GYRO_ZOUT_L            0x48

#define MPU6500_RA_EXT_SENS_DATA_00       0x49
#define MPU6500_RA_EXT_SENS_DATA_01       0x4A
#define MPU6500_RA_EXT_SENS_DATA_02       0x4B
#define MPU6500_RA_EXT_SENS_DATA_03       0x4C
#define MPU6500_RA_EXT_SENS_DATA_04       0x4D
#define MPU6500_RA_EXT_SENS_DATA_05       0x4E
#define MPU6500_RA_EXT_SENS_DATA_06       0x4F
#define MPU6500_RA_EXT_SENS_DATA_07       0x50
#define MPU6500_RA_EXT_SENS_DATA_08       0x51
#define MPU6500_RA_EXT_SENS_DATA_09       0x52
#define MPU6500_RA_EXT_SENS_DATA_10       0x53
#define MPU6500_RA_EXT_SENS_DATA_11       0x54
#define MPU6500_RA_EXT_SENS_DATA_12       0x55
#define MPU6500_RA_EXT_SENS_DATA_13       0x56
#define MPU6500_RA_EXT_SENS_DATA_14       0x57
#define MPU6500_RA_EXT_SENS_DATA_15       0x58
#define MPU6500_RA_EXT_SENS_DATA_16       0x59
#define MPU6500_RA_EXT_SENS_DATA_17       0x5A
#define MPU6500_RA_EXT_SENS_DATA_18       0x5B
#define MPU6500_RA_EXT_SENS_DATA_19       0x5C
#define MPU6500_RA_EXT_SENS_DATA_20       0x5D
#define MPU6500_RA_EXT_SENS_DATA_21       0x5E
#define MPU6500_RA_EXT_SENS_DATA_22       0x5F
#define MPU6500_RA_EXT_SENS_DATA_23       0x60

#define MPU6500_RA_I2C_SLV0_DO            0x63
#define MPU6500_RA_I2C_SLV1_DO            0x64
#define MPU6500_RA_I2C_SLV2_DO            0x65
#define MPU6500_RA_I2C_SLV3_DO            0x66
#define MPU6500_RA_I2C_MST_DELAY_CTRL     0x67

#define MPU6500_RA_SIGNAL_PATH_RESET      0x68
#define MPU6500_RA_ACCEL_INTEL_CTRL       0x69
#define MPU6500_RA_USER_CTRL              0x6A
#define MPU6500_RA_PWR_MGMT_1             0x6B
#define MPU6500_RA_PWR_MGMT_2             0x6C

#define MPU6500_RA_FIFO_COUNTH            0x72
#define MPU6500_RA_FIFO_COUNTL            0x73
#define MPU6500_RA_FIFO_R_W               0x74
#define MPU6500_RA_WHO_AM_I               0x75

#define MPU6500_RA_XA_OFFSET_H            0x77
#define MPU6500_RA_XA_OFFSET_L            0x78
#define MPU6500_RA_YA_OFFSET_H            0x7A
#define MPU6500_RA_YA_OFFSET_L            0x7B
#define MPU6500_RA_ZA_OFFSET_H            0x7D
#define MPU6500_RA_ZA_OFFSET_L            0x7E




#define MPU6500_SMPLRT_DIV_BIT           7       
#define MPU6500_SMPLRT_DIV_LENGTH        8

#define MPU6500_CONFIG_FIFO_MODE_BIT        6 
#define MPU6500_FIFO_NEW_REPLACE_OLD        0x00 
#define MPU6500_FIFO_NEW_NONREPLACE_OLD     0x01 

#define MPU6500_CONFIG_EXT_SYNC_SET_BIT     5 
#define MPU6500_CONFIG_EXT_SYNC_SET_LENGTH  3
#define MPU6500_EXT_SYNC_SET_DISABLE        0x00 
#define MPU6500_EXT_SYNC_SET_TEMP_OUT_L     0x01
#define MPU6500_EXT_SYNC_SET_GYRO_XOUT_L    0x02
#define MPU6500_EXT_SYNC_SET_GYRO_YOUT_L    0x03
#define MPU6500_EXT_SYNC_SET_GYRO_ZOUT_L    0x04
#define MPU6500_EXT_SYNC_SET_ACCEL_XOUT_L   0x05
#define MPU6500_EXT_SYNC_SET_ACCEL_YOUT_L   0x06
#define MPU6500_EXT_SYNC_SET_ACCEL_ZOUT_L   0x07

#define MPU6500_CONFIG_GDLPF_CFG_BIT         2 
#define MPU6500_CONFIG_GDLPF_CFG_LENGTH      3
//when FCHOICE_B <1:0> = 0x00
#define MPU6500_GDLPF_BW_250         0x00
#define MPU6500_GDLPF_BW_184         0x01
#define MPU6500_GDLPF_BW_92          0x02
#define MPU6500_GDLPF_BW_41          0x03
#define MPU6500_GDLPF_BW_20          0x04
#define MPU6500_GDLPF_BW_10          0x05
#define MPU6500_GDLPF_BW_5           0x06
#define MPU6500_GDLPF_BW_3600        0x07

#define MPU6500_GCONFIG_XG_ST_BIT        7
#define MPU6500_GCONFIG_YG_ST_BIT        6
#define MPU6500_GCONFIG_ZG_ST_BIT        5
#define MPU6500_GCONFIG_FS_SEL_BIT       4
#define MPU6500_GCONFIG_FS_SEL_LENGTH    2
#define MPU6500_GCONFIG_FCHOICE_B_BIT    1
#define MPU6500_GCONFIG_FCHOICE_B_LENGTH 2
#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03
#define MPU6500_GCONFIG_FCHOICE_B_BW_8800    0x01
#define MPU6500_GCONFIG_FCHOICE_B_BW_3600    0x02
#define MPU6500_GCONFIG_FCHOICE_B_BW_xxxx    0x00

#define MPU6500_ACONFIG_XA_ST_BIT           7
#define MPU6500_ACONFIG_YA_ST_BIT           6
#define MPU6500_ACONFIG_ZA_ST_BIT           5
#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6500_ACCEL_FS_2          0x00
#define MPU6500_ACCEL_FS_4          0x01
#define MPU6500_ACCEL_FS_8          0x02
#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BIT    3
#define MPU6500_ACONFIG2_ACCEL_FCHOICE_B_LENGTH 1
#define MPU6500_ACONFIG2_A_DLPF_CFG_BIT         2
#define MPU6500_ACONFIG2_A_DLPF_CFG_LENGTH      3
#define MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BW_1130   0x01
//In the low-power mode of operation, the accelerometer is duty-cycled. ACCEL_FCHOICE=0 for all options.
#define MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BW_xxxx   0x00
#define MPU6500_ADLPF_BW_460         0x00
#define MPU6500_ADLPF_BW_184         0x01
#define MPU6500_ADLPF_BW_92          0x02
#define MPU6500_ADLPF_BW_41          0x03
#define MPU6500_ADLPF_BW_20          0x04
#define MPU6500_ADLPF_BW_10          0x05
#define MPU6500_ADLPF_BW_5           0x06
//#define MPU6500_ADLPF_BW_460         0x07


#define MPU6500_LP_AODR_BIT         3
#define MPU6500_LP_AODR_LENGTH      4
#define MPU6500_LP_AODR_BW_0d24          0x00
#define MPU6500_LP_AODR_BW_0d49          0x01
#define MPU6500_LP_AODR_BW_0d98          0x02
#define MPU6500_LP_AODR_BW_1d95          0x03
#define MPU6500_LP_AODR_BW_3d91          0x04
#define MPU6500_LP_AODR_BW_7d81          0x05
#define MPU6500_LP_AODR_BW_15d63         0x06
#define MPU6500_LP_AODR_BW_31d25         0x07
#define MPU6500_LP_AODR_BW_62d50         0x08
#define MPU6500_LP_AODR_BW_125           0x09
#define MPU6500_LP_AODR_BW_250           0x0A
#define MPU6500_LP_AODR_BW_500           0x0B

#define MPU6500_TEMP_FIFO_EN_BIT    7
#define MPU6500_XG_FIFO_EN_BIT      6
#define MPU6500_YG_FIFO_EN_BIT      5
#define MPU6500_ZG_FIFO_EN_BIT      4
#define MPU6500_ACCEL_FIFO_EN_BIT   3
#define MPU6500_SLV2_FIFO_EN_BIT    2
#define MPU6500_SLV1_FIFO_EN_BIT    1
#define MPU6500_SLV0_FIFO_EN_BIT    0

#define MPU6500_MULT_MST_EN_BIT     7
#define MPU6500_WAIT_FOR_ES_BIT     6
#define MPU6500_SLV_3_FIFO_EN_BIT   5
#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4
#define MPU6500_I2C_CLOCK_DIV_348       0x0
#define MPU6500_I2C_CLOCK_DIV_333       0x1
#define MPU6500_I2C_CLOCK_DIV_320       0x2
#define MPU6500_I2C_CLOCK_DIV_308       0x3
#define MPU6500_I2C_CLOCK_DIV_296       0x4
#define MPU6500_I2C_CLOCK_DIV_286       0x5
#define MPU6500_I2C_CLOCK_DIV_276       0x6
#define MPU6500_I2C_CLOCK_DIV_267       0x7
#define MPU6500_I2C_CLOCK_DIV_258       0x8
#define MPU6500_I2C_CLOCK_DIV_500       0x9
#define MPU6500_I2C_CLOCK_DIV_471       0xA
#define MPU6500_I2C_CLOCK_DIV_444       0xB
#define MPU6500_I2C_CLOCK_DIV_421       0xC
#define MPU6500_I2C_CLOCK_DIV_400       0xD
#define MPU6500_I2C_CLOCK_DIV_381       0xE
#define MPU6500_I2C_CLOCK_DIV_364       0xF

#define MPU6500_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6500_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6500_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6500_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6500_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6500_PATHRESET_GYRO_RESET_BIT    2
#define MPU6500_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6500_PATHRESET_TEMP_RESET_BIT    0

#define MPU6500_ACCEL_INTEL_EN_BIT          7
#define MPU6500_ACCEL_INTEL_MODE_BIT        6

#define MPU6500_USERCTRL_DMP_EN_BIT             7
#define MPU6500_USERCTRL_FIFO_EN_BIT            6
#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6500_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6500_USERCTRL_DMP_RESET_BIT          3
#define MPU6500_USERCTRL_FIFO_RESET_BIT         2
#define MPU6500_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6500_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_CYCLE_BIT          5
#define MPU6500_PWR1_GYRO_STANDBY_BIT   4
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3
#define MPU6500_CLOCK_INTERNAL20M       0x00
#define MPU6500_CLOCK_PLL               0x01
//#define MPU6500_CLOCK_PLL               0x02
//#define MPU6500_CLOCK_PLL               0x03
//#define MPU6500_CLOCK_PLL               0x04
//#define MPU6500_CLOCK_PLL               0x05
//#define MPU6500_CLOCK_INTERNAL20M       0x06
#define MPU6500_CLOCK_STOP_KEEP_RESET   0x07

#define MPU6500_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6500_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6500_PWR2_STBY_XA_BIT            5
#define MPU6500_PWR2_STBY_YA_BIT            4
#define MPU6500_PWR2_STBY_ZA_BIT            3
#define MPU6500_PWR2_STBY_XG_BIT            2
#define MPU6500_PWR2_STBY_YG_BIT            1
#define MPU6500_PWR2_STBY_ZG_BIT            0
#define MPU6500_PWR2_STBY_ALL_SENSORS       0x3F
#define MPU6500_PWR2_STBY_XA                0x20
#define MPU6500_PWR2_STBY_YA                0x10
#define MPU6500_PWR2_STBY_ZA                0x08
#define MPU6500_PWR2_STBY_XG                0x04
#define MPU6500_PWR2_STBY_YG                0x02
#define MPU6500_PWR2_STBY_ZG                0x01
#define MPU6500_PWR2_ENABLE_ALL_SENSORS     0x00
#define MPU6500_WAKE_FREQ_1P25      0x0
#define MPU6500_WAKE_FREQ_5         0x1
#define MPU6500_WAKE_FREQ_20        0x2
#define MPU6500_WAKE_FREQ_40        0x3

#define MPU6500_WHO_AM_I_BIT        6
#define MPU6500_WHO_AM_I_LENGTH     6

#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6500_LSB_PER_DEG_250  (float)(65536.0 / (2 * 250.0))
#define MPU6500_LSB_PER_DEG_500  (float)(65536.0 / (2 * 500.0))
#define MPU6500_LSB_PER_DEG_1000 (float)(65536.0 / (2 * 1000.0))
#define MPU6500_LSB_PER_DEG_2000 (float)(65536.0 / (2 * 2000.0))

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)
	
#define MPU6500_LSB_PER_G_2      (float)(65536.0 / (2 * 2))
#define MPU6500_LSB_PER_G_4      (float)(65536.0 / (2 * 4))
#define MPU6500_LSB_PER_G_8      (float)(65536.0 / (2 * 8))
#define MPU6500_LSB_PER_G_16     (float)(65536.0 / (2 * 16))

#define MPU6500_ST_GYRO_LOW      10.0   // deg/s
#define MPU6500_ST_GYRO_HIGH     105.0  // deg/s
#define MPU6500_ST_ACCEL_LOW     0.300  // G
#define MPU6500_ST_ACCEL_HIGH    0.950  // G

 
 
 
 
#define MPU6500_INT_PORT              GPIOA
#define MPU6500_INT_CLK               RCC_AHB1Periph_GPIOA  
#define MPU6500_INT_PIN               GPIO_Pin_4 
#define Set_MPU6500_INT  							MPU6500_INT_PORT->BSRRL = MPU6500_INT_PIN//{GPIO_SetBits(MPU6500_INT_PORT,MPU6500_INT_PIN);}
#define Clr_MPU6500_INT  							MPU6500_INT_PORT->BSRRH = MPU6500_INT_PIN{GPIO_ResetBits(MPU6500_INT_PORT,MPU6500_INT_PIN);} 
#define MPU6500_INT  									(GPIO_ReadInputDataBit(MPU6500_INT_PORT, MPU6500_INT_PIN))




typedef struct
{
    int16_t x;
    int16_t y;
	int16_t z;
}int_3_axis;

typedef struct
{
    float x;
    float y;
	float z;
}float_3_axis;



rt_err_t MPU6500_Init(void);

void MPU6500_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
						int16_t* gx, int16_t* gy, int16_t* gz);

float MPU6500_GetFullScaleGyroDPL(void);
float MPU6500_GetFullScaleAccelGPL(void);
rt_err_t mpu6500_connect_test(void);
#endif

