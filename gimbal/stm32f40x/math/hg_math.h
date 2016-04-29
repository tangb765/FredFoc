// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __HG_MATH_H__
#define __HG_MATH_H__

#include <stdint.h>
#include <math.h>
#include <float.h>

#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif

//Single precision conversions
#ifndef DEG_TO_RAD
	#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif

#ifndef RAD_TO_DEG
	#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v);

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v);
// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v);

#define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
// fast_atan2 - faster version of atan2
//      126 us on AVR cpu vs 199 for regular atan2
//      absolute error is < 0.005 radians or 0.28 degrees
//      origin source: https://gist.github.com/volkansalma/2972237/raw/
float fast_atan2(float y, float x);


// constrain a value
float constrain_float(float amt, float low, float high);
// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
// degrees -> radians
float radians(float deg);
// radians -> degrees
float degrees(float rad);
// square
float sq(float v);
// 2D vector length
float pythagorous2(float a, float b);
// 3D vector length
float pythagorous3(float a, float b, float c);

float min_float(float val1, float val2);
float max_float(float val1, float val2);
float constrain(float val, float min, float max);

#endif // HG_MATH_H
