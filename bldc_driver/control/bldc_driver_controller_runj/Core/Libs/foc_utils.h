#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include <cstdint>  // for uint32_t
#include <math.h>
#include <stdlib.h>
#include <algorithm>
using namespace std;

// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC (NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)

enum Direction {
	CW = 1,  //clockwise
	CCW = -1, // counter clockwise
	UNKNOWN = 0   //not yet known or invalid state
};

struct DQCurrent_s // dq current structure
{
	float d;
	float q;
};
struct PhaseCurrent_s // phase current structure
{
	float a;
	float b;
	float c;
};
struct DQVoltage_s // d-q voltage struct
{
	float d;
	float q;
};

/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _sin(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _cos(float a);
/**
 * Function returning both sine and cosine of the angle in one call.
 * Internally it uses the _sin and _cos functions, but you may wish to
 * provide your own implementation which is more optimized.
 */
void _sincos(float a, float* s, float* c);

/**
 * Function approximating atan2
 *
 */
float _atan2(float y, float x);

/**
 * normalizing radian angle to [0,2PI]
 * @param angle - angle to be normalized
 */
float _normalizeAngle(float angle);


/**
 * Electrical angle calculation
 *
 * @param shaft_angle - shaft angle of the motor
 * @param pole_pairs - number of pole pairs
 */
float _electricalAngle(float shaft_angle, int pole_pairs);

/**
 * Function approximating square root function
 *  - using fast inverse square root
 *
 * @param value - number
 */
float _sqrtApprox(float value);

#endif