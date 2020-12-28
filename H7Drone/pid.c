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

	pidData[PID_ROLL].Kp = PTERM_SCALE * 42;
	pidData[PID_ROLL].Ki = ITERM_SCALE * 85;
	pidData[PID_ROLL].Kd = DTERM_SCALE * 35;

	pidData[PID_PITCH].Kp = PTERM_SCALE * 46;
	pidData[PID_PITCH].Ki = ITERM_SCALE * 90;
	pidData[PID_PITCH].Kd = DTERM_SCALE * 38;

	pidData[PID_YAW].Kp = PTERM_SCALE * 45;
	pidData[PID_YAW].Ki = ITERM_SCALE * 90;
	pidData[PID_YAW].Kd = DTERM_SCALE * 0;
}

void FAST_CODE pidUpdate(timeUs_t currentTimeUs)
{
	for (int i = 0; i < PID_ITEM_COUNT; i++)
	{
		pidData_t *pid = &pidData[i];

		float setpoint = getSetpointRate(i);
		float error = setpoint - imuData.gyroData.xyz[i];
		float proportional = pid->Kp * error;

		pid->integrator = constrainf(pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError), -pidRuntime.itermLimit, pidRuntime.itermLimit);
		pid->differentiator = -(2.0f * pid->Kd * (setpoint - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / (2.0f * pid->tau + pid->T);

		pid->sum = constrainf(proportional + pid->integrator + pid->differentiator, pid->limMin, pid->limMax);
		
		pid->prevError = error;
		pid->prevMeasurement = setpoint;
	}
}
