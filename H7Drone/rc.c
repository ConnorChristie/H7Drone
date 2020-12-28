#include "rc.h"
#include "sbus.h"
#include "control.h"

float channelData[MAX_CHANNEL_COUNT];

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	sbusReadRawRC(channelData);

	// Current controller is setup for TAER
	setThrottle(channelData[0]);
	updateSetpointRate(CONTROL_ROLL, channelData[1]);
	updateSetpointRate(CONTROL_PITCH, channelData[2]);
	updateSetpointRate(CONTROL_YAW, channelData[3]);
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
	const u8 frameStatus = sbusFrameStatus();
	bool signalReceived = false;

	if (frameStatus & RX_FRAME_COMPLETE)
	{
		bool rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
		bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;

		return !(rxIsInFailsafeMode || rxFrameDropped);
	}

	return false;
}
