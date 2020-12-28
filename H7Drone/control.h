#pragma once

#include "platform.h"

typedef enum {
	CONTROL_ROLL,
	CONTROL_PITCH,
	CONTROL_YAW,
	CONTROL_ITEM_COUNT
} controlIndex_e;

bool isArmed(void);
void setArmed(bool isArmed);

float getThrottle(void);
void setThrottle(float value);

float getSetpointRate(int axis);
void updateSetpointRate(int axis, float setpoint);
