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
	CONTROL_ITEM_COUNT
} controlIndex_e;

extern float channelData[MAX_CHANNEL_COUNT];

typedef struct
{
	void (*init)(void);
	void(*readRawRc)(float *channelData);
	u8 (*readStatus)(void);
} controlVtable_t;

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void taskUpdateRxMain(timeUs_t currentTimeUs);

bool isArmed(void);
void setArmed(bool isArmed);

float getThrottle(void);
void setThrottle(float value);

float getSetpointRate(int axis);
void updateSetpointRate(int axis, float setpoint);
