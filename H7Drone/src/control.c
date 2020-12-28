#include "control.h"
#include "maths.h"

#include "sbus.h"

FAST_DATA_ZERO_INIT bool armed;
FAST_DATA_ZERO_INIT float setpoints[CONTROL_ITEM_COUNT];
FAST_DATA_ZERO_INIT float controlData[MAX_CHANNEL_COUNT];

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
	controlVTable.readRawRc(&controlData[0], MAX_CHANNEL_COUNT);

	// Current controller is setup for TAER
	updateSetpointRate(CONTROL_THROTTLE, controlData[0]);
	updateSetpointRate(CONTROL_ROLL, controlData[1]);
	updateSetpointRate(CONTROL_PITCH, controlData[2]);
	updateSetpointRate(CONTROL_YAW, controlData[3]);

	setArmed(controlData[6] >= 1500);
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
	if (armed == isArmed) return;

	if (getSetpointRate(CONTROL_THROTTLE) <= -1.0f && isArmed)
	{
		armed = true;
		return;
	}

	armed = false;
}

float getSetpointRate(int axis)
{
	return setpoints[axis];
}

void updateSetpointRate(int axis, float setpoint)
{
	float rcCommandf = setpoint - 1500;
	rcCommandf = rcCommandf / 500.0f;

	setpoints[axis] = rcCommandf;
}
