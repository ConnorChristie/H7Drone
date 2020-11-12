#pragma once

#include <stdint.h>

#define sq(x) ((x)*(x))

#define ACCEL_1G 9.80665f
#define M_PIf 3.14159265358979323846f
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })

float atan2_approx(float y, float x);

typedef union
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};

	int16_t xyz[3];
} vector3_t;

typedef union
{
	struct
	{
		float x;
		float y;
		float z;
	};

	float xyz[3];
} vector3f_t;

typedef union
{
	struct
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};

	int64_t xyz[3];
} vector3l_t;

typedef struct {
	float w, x, y, z;
} quaternion_t;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
	float ww, wx, wy, wz, xx, xy, xz, yy, yz, zz;
} quaternionProducts_t;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef struct stdev_s
{
	float m_oldM, m_newM, m_oldS, m_newS;
	int m_n;
} stdev_t;

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);

static inline int constrain(int amt, int low, int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
