#include "pid.h"
#include "platform.h"
#include "control.h"
#include "imu.h"
#include "maths.h"

FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;
FAST_DATA_ZERO_INIT pidData_t pidData[PID_ITEM_COUNT];

#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

void pidInit(void)
{
	pidRuntime.itermLimit = 400;
	pidRuntime.tau = 0.02f;

	pidData[PID_ROLL].pidSumLimit = 500;
	pidData[PID_ROLL].Kp = PTERM_SCALE * 42;
	pidData[PID_ROLL].Ki = ITERM_SCALE * 85;
	pidData[PID_ROLL].Kd = DTERM_SCALE * 35;

	pidData[PID_PITCH].pidSumLimit = 500;
	pidData[PID_PITCH].Kp = PTERM_SCALE * 46;
	pidData[PID_PITCH].Ki = ITERM_SCALE * 90;
	pidData[PID_PITCH].Kd = DTERM_SCALE * 38;

	pidData[PID_YAW].pidSumLimit = 400;
	pidData[PID_YAW].Kp = PTERM_SCALE * 45;
	pidData[PID_YAW].Ki = ITERM_SCALE * 90;
	pidData[PID_YAW].Kd = DTERM_SCALE * 0;
}

timeUs_t prevTimePid = 0;
vector3f_t pidErrors;

void FAST_CODE pidUpdate(timeUs_t currentTimeUs)
{
	if (prevTimePid == 0)
	{
		prevTimePid = currentTimeUs;
		return;
	}

	float dt = (currentTimeUs - prevTimePid) * 1e-6f;
	prevTimePid = currentTimeUs;

	for (int i = 0; i < PID_ITEM_COUNT; i++)
	{
		pidData_t *pid = &pidData[i];

		float setpoint = getSetpointRate(i);
		float error = setpoint - imuData.gyroData.xyz[i];
		float ff = constrainf(pidRuntime.tau * setpoint, 0, 0);

		pidErrors.xyz[i] = error;

		pid->P = pid->Kp * error;
		pid->I = pid->I + pid->Ki * dt * (error + pid->prevError);

		/* Dynamic integrator clamping */
		float limMinInt, limMaxInt;

		if (pid->pidSumLimit - pid->P - ff > 0.0f)
		{
			limMaxInt = pid->pidSumLimit - pid->P;
		}
		else
		{
			limMaxInt = 0.0f;
		}

		if (-pid->pidSumLimit - pid->P - ff < 0.0f)
		{
			limMinInt = -pid->pidSumLimit - pid->P;
		}
		else
		{
			limMinInt = 0.0f;
		}

		pid->I = constrainf(pid->I, limMinInt, limMaxInt);
		//pid->D = -(2.0f * pid->Kd * (setpoint - pid->prevMeasurement) + (2.0f * pidRuntime.tau - dt) * pid->D) / (2.0f * pidRuntime.tau + dt);

		pid->output = constrainf(pid->P + ff + pid->I, -pid->pidSumLimit, pid->pidSumLimit);
		
		pid->prevError = error;
		//pid->prevMeasurement = setpoint;
	}

	if (getSetpointRate(CONTROL_THROTTLE) <= -666)
	{
		pidResetITerm();
	}
}

void pidReset(void)
{
	for (int i = 0; i < PID_ITEM_COUNT; i++)
	{
		pidData_t *pid = &pidData[i];

		pid->P = 0;
		pid->I = 0;
		pid->D = 0;
		pid->prevError = 0;
		pid->output = 0;
	}
}

void pidResetITerm(void)
{
	for (int i = 0; i < PID_ITEM_COUNT; i++)
	{
		pidData_t *pid = &pidData[i];

		pid->I = 0;
	}
}
