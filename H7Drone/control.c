#include "control.h"
#include "maths.h"

#include "sbus.h"

FAST_DATA_ZERO_INIT bool armed;
FAST_DATA_ZERO_INIT float throttle;
FAST_DATA_ZERO_INIT vector3f_t setpoints;

FAST_DATA_ZERO_INIT float channelData[MAX_CHANNEL_COUNT];

static controlVtable_t controlVTable;

void initControl(void)
{
	controlVTable.init = sbusInit;
	controlVTable.readRawRc = sbusReadRawRC;
	controlVTable.readStatus = sbusFrameStatus;

	controlVTable.init();
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	controlVTable.readRawRc(&channelData[0]);

	// Current controller is setup for TAER
	setThrottle(channelData[0]);
	updateSetpointRate(CONTROL_ROLL, channelData[1]);
	updateSetpointRate(CONTROL_PITCH, channelData[2]);
	updateSetpointRate(CONTROL_YAW, channelData[3]);
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
	const u8 frameStatus = controlVTable.readStatus();
	bool signalReceived = false;

	if (frameStatus & RX_FRAME_COMPLETE)
	{
		bool rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
		bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;

		return !(rxIsInFailsafeMode || rxFrameDropped);
	}

	return false;
}

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
