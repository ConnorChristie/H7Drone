#pragma once

#include "platform.h"
#include "control.h"

typedef enum
{
	PID_ROLL   = CONTROL_ROLL,
	PID_PITCH  = CONTROL_PITCH,
	PID_YAW    = CONTROL_YAW,
	PID_ITEM_COUNT
} pidIndex_e;

typedef struct
{
	/* Integrator limits */
	float itermLimit;
} pidRuntime_t;

typedef struct
{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError; /* Required for integrator */
	float differentiator;
	float prevMeasurement; /* Required for differentiator */

	float sum;
} pidData_t;

extern pidData_t pidData[PID_ITEM_COUNT];

void pidInit(void);
void pidUpdate(timeUs_t currentTimeUs);
