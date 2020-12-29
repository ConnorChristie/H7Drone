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
	/* Sample time (in seconds) */
	float T;
	/* Derivative low-pass filter time constant */
	float tau;
} pidRuntime_t;

typedef struct
{
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float pidSumLimit;

	float P;
	float I;
	float D;

	float prevError;
	float differentiator;
	float prevMeasurement;

	float output;
} pidData_t;

extern pidData_t pidData[PID_ITEM_COUNT];

void pidInit(void);
void pidUpdate(timeUs_t currentTimeUs);
void pidReset(void);
void pidResetITerm(void);
