#include "control.h"
#include "maths.h"

FAST_DATA_ZERO_INIT bool armed;
FAST_DATA_ZERO_INIT float throttle;
FAST_DATA_ZERO_INIT vector3f_t setpoints;

bool isArmed(void)
{
	return armed;
}

void setArmed(bool isArmed)
{
	if (throttle == 0 && isArmed)
	{
		armed = true;
	}

	armed = false;
}

float getThrottle(void)
{
	return throttle;
}

void setThrottle(float value)
{
	throttle = value;
}

float getSetpointRate(int axis)
{
	return setpoints.xyz[axis];
}

void updateSetpointRate(int axis, float setpoint)
{
	float rcCommandf = setpoint - 1500;
	rcCommandf = rcCommandf / 500.0f;

	setpoints.xyz[axis] = rcCommandf;
}
