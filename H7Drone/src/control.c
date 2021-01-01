#include "control.h"
#include "maths.h"
#include "pid.h"

#include "sbus.h"

FAST_DATA_ZERO_INIT bool armed;
FAST_DATA_ZERO_INIT float rcData[CONTROL_ITEM_COUNT];
FAST_DATA_ZERO_INIT float rcCommand[CONTROL_ITEM_COUNT];
FAST_DATA_ZERO_INIT float controlData[MAX_CHANNEL_COUNT];

static controlVtable_t controlVTable;

static float minRc = 1050, midRc = 1500, maxRc = 1900;

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs);

void initControl(void)
{
	controlVTable.init = sbusInit;
	controlVTable.readRawRc = sbusReadRawRC;
	controlVTable.readStatus = sbusFrameStatus;
	controlVTable.applyRates = applyBetaflightRates;

	controlVTable.init();
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	controlVTable.readRawRc(&controlData[0], MAX_CHANNEL_COUNT);

	// Current controller is setup for TAER
	rcData[CONTROL_THROTTLE] = controlData[0];
	rcData[CONTROL_ROLL] = controlData[1];
	rcData[CONTROL_PITCH] = controlData[2];
	rcData[CONTROL_YAW] = controlData[3];

	for (int i = 0; i < CONTROL_ITEM_COUNT; i++)
	{
		rcCommand[i] = constrainf(rcData[i] - midRc, -500, 500) / 500.0f;

		if (i == CONTROL_THROTTLE)
		{
			rcCommand[i] = (rcCommand[i] + 1.0f) / 2.0f;
		}
		else
		{
			rcCommand[i] = controlVTable.applyRates(i, rcCommand[i], fabsf(rcCommand[i]));
		}
	}

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

bool FAST_CODE isArmed(void)
{
	return armed;
}

bool FAST_CODE isThrottleDown(void)
{
	return getSetpointRate(CONTROL_THROTTLE) <= 0.0f;
}

void setArmed(bool isArmed)
{
	if (armed == isArmed) return;

	if (isThrottleDown() && isArmed)
	{
		armed = true;

		pidReset();

		return;
	}

	armed = false;
}

float FAST_CODE getSetpointRate(int axis)
{
	return rcCommand[axis];
}

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
	float rcRate = 1.0f;
	float angleRate = 200.0f * rcRate * rcCommandf;

	const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * 0.70f), 0.01f, 1.00f));
	angleRate *= rcSuperfactor;

	return angleRate;
}
