#pragma once

#include "platform.h"

#define MAX_CHANNEL_COUNT 18

typedef enum
{
	RX_FRAME_PENDING             = 0,
	RX_FRAME_COMPLETE            = (1 << 0),
	RX_FRAME_FAILSAFE            = (1 << 1),
	RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
	RX_FRAME_DROPPED             = (1 << 3)
} rxFrameState_e;

extern float channelData[MAX_CHANNEL_COUNT];

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void taskUpdateRxMain(timeUs_t currentTimeUs);
