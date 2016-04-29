/*
 * File      : sensfusion6.c
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

#include "stm32f4xx.h"
#include <math.h>

#include "sensfusion6.h"

//#define MADWICK_QUATERNION_IMU

#define M_PI 3.1415926f

#ifdef MADWICK_QUATERNION_IMU
#define BETA_DEF     0.01f    // 2 * proportional gain
#else // MAHONY_QUATERNION_IMU
#define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
#endif

#ifdef MADWICK_QUATERNION_IMU
float beta = BETA_DEF;     // 2 * proportional gain (Kp)
#else // MAHONY_QUATERNION_IMU
float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki
#endif

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

static bool isInit;

// TODO: Make math util file
static float invSqrt(float x);

//kalmanfilter halferr[3];
//kalmanfilter halfgyro[3];

void sensfusion6Init()
{
    uint8_t i;
    if(isInit)
        return;

//    for (i = 0; i < 3; i++)
//    {
//        //		KalmanFilter_Init(&halferr[i], 20, 10);
//        KalmanFilter_Init(&halferr[i], 10, 1000);
//        KalmanFilter_Init(&halfgyro[i], 20, 10);
//    }

    isInit = true;
}

bool sensfusion6Test(void)
{
    return isInit;
}

void initQ(float Roll, float Pitch, float Yaw)
{
    float fRoll, fPitch, fYaw;
    float fCOSY, fCOSP, fCOSR;
    float fSINY, fSINP, fSINR;

    //
    // Convert roll, pitch, and yaw from degrees into radians
    //
    //    fRoll = fRollDeg * M_PI / 180.0f;
    //    fPitch = fPitchDeg *  M_PI / 180.0f;
    //    fYaw = fYawDeg * M_PI / 180.0f;

    //
    // Pre-calculate the cosine of (yaw, pitch, roll divided by 2)
    //
    fCOSY = cosf(Yaw / 2.0f);
    fCOSP = cosf(Pitch / 2.0f);
    fCOSR = cosf(Roll / 2.0f);

    //
    // Pre-calculate the sine of (yaw, pitch, roll divided by 2)
    //
    fSINY = sinf(fYaw / 2.0f);
    fSINP = sinf(fPitch / 2.0f);
    fSINR = sinf(fRoll / 2.0f);

    q0 = fCOSY * fCOSP * fCOSR + fSINY * fSINP * fSINR;
    q1 = fCOSY * fCOSP * fSINR - fSINY * fSINP * fCOSR;//      fSINY * fSINP * fCOSR + fCOSY * fCOSP * fSINR;
    q2 = fCOSY * fSINP * fCOSR + fSINY * fCOSP * fSINR;
    q3 = fSINY * fCOSP * fCOSR - fCOSY * fSINP * fSINR;
}


#ifdef MADWICK_QUATERNION_IMU
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
#else // MAHONY_QUATERNION_IMU
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
uint8_t micro_angle_flag = 0;
float recipNormG_T = 0.15f;

float twoKpk = 2.0f;//50;//1.6;//50;//2*TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKik = 0.002;//10;//0.002;//10;//TWO_KI_DEF;    // 2 * integral gain (Ki)

float twoKpd = 0.15;
float twoKid = 0.002;
float halfex, halfey, halfez;
float halfex1, halfey1, halfez1;
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;

    float qa, qb, qc;

    gx = gx * M_PI / 180.0f;
    gy = gy * M_PI / 180.0f;
    gz = gz * M_PI / 180.0f;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        if (micro_angle_flag)   //recipNormG<1.003 && recipNormG>0.997
        {
            micro_angle_flag = 0;
            gx = 0;
            gy = 0;
            gz = 0;
            twoKi = twoKik;
            twoKp = twoKpk;
        }
        else
        {
            twoKi = twoKid;
            twoKp = twoKpd;
        }

        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex1 = (ay * halfvz - az * halfvy);
        halfey1 = (az * halfvx - ax * halfvz);
        halfez1 = (ax * halfvy - ay * halfvx);

//        halfex = KalmanFilter(&halferr[0], halfex1);
//        halfey = KalmanFilter(&halferr[1], halfey1);
//        halfez = KalmanFilter(&halferr[2], halfez1);


        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f)// && recipNorm>(1-recipNormG_T) && recipNorm <(1+recipNormG_T))
        {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        //		if(recipNorm>(1-recipNormG_T) && recipNorm <(1+recipNormG_T))
        {
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
    }



    //	gx = KalmanFilter(&halfgyro[0], gx);
    //	gy = KalmanFilter(&halfgyro[1], gy);
    //	gz = KalmanFilter(&halfgyro[2], gz);

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);   // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
#endif


//void sensfusiondebug(void)
//{
//    static uint32_t count = 0;

//    if (count % 10 == 0)
//    {
//        Debug_write(1, "fusion:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//                    (rt_int32_t)(halfex1 * 10000),
//                    (rt_int32_t)(halfey1 * 10000),
//                    (rt_int32_t)(halfez1 * 10000),
//                    (rt_int32_t)(halfex * 10000),
//                    (rt_int32_t)(halfey * 10000),
//                    (rt_int32_t)(halfez * 10000),
//                    //		(rt_int32_t)(gx*1000),
//                    //		(rt_int32_t)(gy*1000),
//                    //		(rt_int32_t)(gz*1000),
//                    (rt_int32_t)(q0 * 10000),
//                    (rt_int32_t)(q1 * 10000),
//                    (rt_int32_t)(q2 * 10000),
//                    (rt_int32_t)(q3 * 10000));
//    }
//}


//float gx1,  gy1,  gz1;
//float gx2,  gy2,  gz2;
//void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
//{
//  float recipNorm;
//  float halfvx, halfvy, halfvz;
//
//  float qa, qb, qc;

//  gx1 = gx * M_PI / 180;
//  gy1 = gy * M_PI / 180;
//  gz1 = gz * M_PI / 180;
//
//  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//  {
//    // Normalise accelerometer measurement
//    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		if (micro_angle_flag)   //recipNormG<1.003 && recipNormG>0.997
//		{
//			micro_angle_flag = 0;
//			gx1 = 0;
//			gy1 = 0;
//			gz1 = 0;
//			twoKi = twoKik;
//			twoKp = twoKpk;
//		}
//		else
//		{
//			twoKi = twoKik;
//			twoKp = twoKpk;
//		}
//
//    ax *= recipNorm;
//    ay *= recipNorm;
//    az *= recipNorm;

//    // Estimated direction of gravity and vector perpendicular to magnetic flux
//    halfvx = q1 * q3 - q0 * q2;
//    halfvy = q0 * q1 + q2 * q3;
//    halfvz = q0 * q0 - 0.5f + q3 * q3;

//    // Error is sum of cross product between estimated and measured direction of gravity
//    halfex1 = (ay * halfvz - az * halfvy);
//    halfey1 = (az * halfvx - ax * halfvz);
//    halfez1 = (ax * halfvy - ay * halfvx);

////		halfex = KalmanFilter(&halferr[0], halfex1);
////		halfey = KalmanFilter(&halferr[1], halfey1);
////		halfez = KalmanFilter(&halferr[2], halfez1);
//
//
//    // Compute and apply integral feedback if enabled
//		if(twoKi > 0.0f)// && recipNorm>(1-recipNormG_T) && recipNorm <(1+recipNormG_T))
//    {
//      integralFBx += twoKi * halfex * dt;
//      integralFBy += twoKi * halfey * dt;
//      integralFBz += twoKi * halfez * dt;
//      gx1 += integralFBx;
//      gy1 += integralFBy;
//      gz1 += integralFBz;
//    }
//    else
//    {
//      integralFBx = 0.0f; // prevent integral windup
//      integralFBy = 0.0f;
//      integralFBz = 0.0f;
//    }

//    // Apply proportional feedback
////		if(recipNorm>(1-recipNormG_T) && recipNorm <(1+recipNormG_T))
//		{
//			gx1 += twoKp * halfex;
//			gy1 += twoKp * halfey;
//			gz1 += twoKp * halfez;
//		}
//  }
//
//

//	gx2 = KalmanFilter(&halferr[0], gx1);
//	gy2 = KalmanFilter(&halferr[1], gy1);
//	gz2 = KalmanFilter(&halferr[2], gz1);

//  // Integrate rate of change of quaternion
//  gx2 *= (0.5f * dt);   // pre-multiply common factors
//  gy2 *= (0.5f * dt);
//  gz2 *= (0.5f * dt);
//  qa = q0;
//  qb = q1;
//  qc = q2;
//  q0 += (-qb * gx2 - qc * gy2 - q3 * gz2);
//  q1 += (qa * gx2 + qc * gz2 - q3 * gy2);
//  q2 += (qa * gy2 - qb * gz2 + q3 * gx2);
//  q3 += (qa * gz2 + qb * gy2 - qc * gx2);

//  // Normalise quaternion
//  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//  q0 *= recipNorm;
//  q1 *= recipNorm;
//  q2 *= recipNorm;
//  q3 *= recipNorm;
//}
//#endif


//void sensfusiondebug(void)
//{
//	static uint32_t count = 0;

//	if (count%10==0)
//	{
//		Debug_write(1,"fusion:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
////		(rt_int32_t)(halfex1*10000),
////		(rt_int32_t)(halfey1*10000),
////		(rt_int32_t)(halfez1*10000),
////		(rt_int32_t)(halfex*10000),
////		(rt_int32_t)(halfey*10000),
////		(rt_int32_t)(halfez*10000),
//		(rt_int32_t)(gx1*1000),
//		(rt_int32_t)(gy1*1000),
//		(rt_int32_t)(gz1*1000),
//		(rt_int32_t)(gx2*1000),
//		(rt_int32_t)(gy2*1000),
//		(rt_int32_t)(gz2*1000),
//		(rt_int32_t)(q0*10000),
//		(rt_int32_t)(q1*10000),
//		(rt_int32_t)(q2*10000),
//		(rt_int32_t)(q3*10000));
//	}
//}

void sensfusion6GetEulerRPY(float *roll, float *pitch, float *yaw)
{
    float gx, gy, gz; // estimated gravity direction

    gx = 2 * (q1 * q3 - q0 * q2);
    gy = 2 * (q0 * q1 + q2 * q3);
    gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / M_PI;
    *pitch = asin(gx) * 180 / M_PI; //Pitch seems to be inverted
    *roll = atan2(gy, gz) * 180 / M_PI;
}

float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
    float gx, gy, gz; // estimated gravity direction

    gx = 2 * (q1 * q3 - q0 * q2);
    gy = 2 * (q0 * q1 + q2 * q3);
    gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // return vertical acceleration without gravity
    // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
    return ((ax * gx + ay * gy + az * gz) - 1.0);
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

