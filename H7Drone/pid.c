#include "pid.h"
#include "platform.h"
#include "control.h"
#include "imu.h"
#include "maths.h"

FAST_DATA_ZERO_INIT pidData_t pidData[3];

void FAST_CODE pidUpdate(timeUs_t currentTimeUs)
{
	for (int i = 0; i < 3; i++)
	{
		pidData_t *pid = &pidData[i];

		float setpoint = getSetpointRate(i);
		float error = setpoint - imuData.gyroData.xyz[i];
		float proportional = pid->Kp * error;

		pid->integrator = constrainf(pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError), -pid->itermLimit, pid->itermLimit);
		pid->differentiator = -(2.0f * pid->Kd * (setpoint - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / (2.0f * pid->tau + pid->T);

		pid->sum = constrainf(proportional + pid->integrator + pid->differentiator, pid->limMin, pid->limMax);
		
		pid->prevError = error;
		pid->prevMeasurement = setpoint;
	}
}
