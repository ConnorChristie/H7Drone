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

typedef enum {
	CONTROL_ROLL,
	CONTROL_PITCH,
	CONTROL_YAW,
	CONTROL_THROTTLE,
	CONTROL_ITEM_COUNT
} controlIndex_e;

typedef struct
{
	void (*init)(void);
	void (*readRawRc)(float *channelData, int count);
	u8 (*readStatus)(void);
	float (*applyRates)(const int index, float rcCommandf, const float rcCommandfAbs);
} controlVtable_t;

extern float controlData[MAX_CHANNEL_COUNT];

void initControl(void);

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void taskUpdateRxMain(timeUs_t currentTimeUs);

bool isThrottleDown(void);
bool isArmed(void);
void setArmed(bool isArmed);

float getSetpointRate(int axis);
