/*
 * File      : sensfusion6.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-10-14     Fred.deng    the first version
 */

#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include <stdbool.h>

void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
void initQ(float Roll, float Pitch, float Yaw);

#endif /* SENSORFUSION6_H_ */
