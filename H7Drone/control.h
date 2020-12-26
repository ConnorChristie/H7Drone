#pragma once

#include "platform.h"

bool isArmed(void);
void setArmed(bool isArmed);

float getThrottle(void);
void setThrottle(float value);

float getSetpointRate(int axis);
void updateSetpointRate(int axis, float setpoint);
