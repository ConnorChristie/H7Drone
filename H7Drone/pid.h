#pragma once

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

	/* Integrator limits */
	float itermLimit;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError; /* Required for integrator */
	float differentiator;
	float prevMeasurement; /* Required for differentiator */

	float sum;
} pidData_t;

void pidUpdate();
