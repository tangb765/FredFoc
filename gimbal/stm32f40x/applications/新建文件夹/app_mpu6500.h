#ifndef __APP_MPU6500_H_
#define __APP_MPU6500_H_

#include "rtthread.h"
//#include "uorb.h"
//#include "drivers.h"
#include "stdbool.h"

#define GRAVITY   9.7925f 
#define DEGTORAD  0.01748081f



void app_mpu6500_main(void* parameter);

#endif
