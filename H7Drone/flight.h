#pragma once

#include "scheduler.h"

typedef struct
{
	float p;
	float i;
	float d;
} pidGains_t;

void flightUpdate(timeUs_t currentTimeUs);
