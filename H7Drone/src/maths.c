#include <math.h>
#include "maths.h"

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
	#define atanPolyCoef1  3.14551665884836e-07f
	#define atanPolyCoef2  0.99997356613987f
	#define atanPolyCoef3  0.14744007058297684f
	#define atanPolyCoef4  0.3099814292351353f
	#define atanPolyCoef5  0.05030176425872175f
	#define atanPolyCoef6  0.1471039133652469f
	#define atanPolyCoef7  0.6444640676891548f

	float res, absX, absY;
	absX = fabsf(x);
	absY = fabsf(y);
	res  = MAX(absX, absY);
	if (res) res = MIN(absX, absY) / res;
	else res = 0.0f;
	res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
	if (absY > absX) res = (M_PIf / 2.0f) - res;
	if (x < 0) res = M_PIf - res;
	if (y < 0) res = -res;
	return res;
}

void devClear(stdev_t *dev)
{
	dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
	dev->m_n++;
	if (dev->m_n == 1) {
		dev->m_oldM = dev->m_newM = x;
		dev->m_oldS = 0.0f;
	}
	else {
		dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
		dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
		dev->m_oldM = dev->m_newM;
		dev->m_oldS = dev->m_newS;
	}
}

float devVariance(stdev_t *dev)
{
	return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
	return sqrtf(devVariance(dev));
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
