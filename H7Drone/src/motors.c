#include "motors.h"
#include "system.h"
#include "control.h"
#include "maths.h"
#include "pid.h"

#include "motors/dshot.h"

FAST_DATA_ZERO_INIT u16 motorValues[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT motorInstance_t motors[MAX_SUPPORTED_MOTORS];

static motorsVtable_t motorsVtable;

void initMotors(void)
{
	motorsVtable.init = dshotInit;
	motorsVtable.writeMotors = dshotWriteMotors;

	motorInstance_t *motor = &motors[0];

	motor->timer.pinPack = GPIOB;
	motor->timer.pin = GPIO_PIN_1;
	motor->timer.alternateFunction = GPIO_AF2_TIM3;
	motor->timer.instance = TIM3;
	motor->timer.channel = TIM_CHANNEL_4;

	motor->dma = DMA1_ST0_HANDLER;
	motor->dmaChannel = LL_DMAMUX1_REQ_TIM3_CH4;

	motorsVtable.init(0, motor);

	motor = &motors[1];
	motor->timer.pinPack = GPIOA;
	motor->timer.pin = GPIO_PIN_0;
	motor->timer.alternateFunction = GPIO_AF2_TIM5;
	motor->timer.instance = TIM5;
	motor->timer.channel = TIM_CHANNEL_1;

	motor->dma = DMA1_ST1_HANDLER;
	motor->dmaChannel = LL_DMAMUX1_REQ_TIM5_CH1;

	motorsVtable.init(1, motor);

	motor = &motors[2];
	motor->timer.pinPack = GPIOA;
	motor->timer.pin = GPIO_PIN_1;
	motor->timer.alternateFunction = GPIO_AF2_TIM5;
	motor->timer.instance = TIM5;
	motor->timer.channel = TIM_CHANNEL_2;

	motor->dma = DMA1_ST2_HANDLER;
	motor->dmaChannel = LL_DMAMUX1_REQ_TIM5_CH2;

	motorsVtable.init(2, motor);

	motor = &motors[3];
	motor->timer.pinPack = GPIOA;
	motor->timer.pin = GPIO_PIN_2;
	motor->timer.alternateFunction = GPIO_AF2_TIM5;
	motor->timer.instance = TIM5;
	motor->timer.channel = TIM_CHANNEL_3;

	motor->dma = DMA1_ST3_HANDLER;
	motor->dmaChannel = LL_DMAMUX1_REQ_TIM5_CH3;

	motorsVtable.init(3, motor);
}

void motorsUpdate(timeUs_t currentTimeUs)
{
	// 0 - 2047

	static float mixer[MAX_SUPPORTED_MOTORS][PID_ITEM_COUNT];

	mixer[0][PID_ROLL] = -1;
	mixer[1][PID_ROLL] = -1;
	mixer[2][PID_ROLL] = 1;
	mixer[3][PID_ROLL] = 1;

	mixer[0][PID_PITCH] = 1;
	mixer[1][PID_PITCH] = -1;
	mixer[2][PID_PITCH] = 1;
	mixer[3][PID_PITCH] = -1;

	mixer[0][PID_YAW] = -1;
	mixer[1][PID_YAW] = 1;
	mixer[2][PID_YAW] = 1;
	mixer[3][PID_YAW] = -1;

	if (isArmed())
	{
		float throttle = (getSetpointRate(CONTROL_THROTTLE) + 666.0f) / (666.0f * 2.0f);

		const float scaledAxisPidRoll = pidData[PID_ROLL].output / 1000.0f;
		const float scaledAxisPidPitch = pidData[PID_PITCH].output / 1000.0f;
		const float scaledAxisPidYaw = pidData[PID_YAW].output / 1000.0f;

		float motorMix[MAX_SUPPORTED_MOTORS];
		float motorMixMax = 0, motorMixMin = 0;

		for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
		{
			float mix =
				mixer[i][PID_ROLL] * scaledAxisPidRoll +
				mixer[i][PID_PITCH] * scaledAxisPidPitch +
				mixer[i][PID_YAW] * scaledAxisPidYaw;

			if (mix > motorMixMax)
			{
				motorMixMax = mix;
			}
			else if (mix < motorMixMin)
			{
				motorMixMin = mix;
			}

			motorMix[i] = mix;
		}

		throttle = constrainf(throttle, -motorMixMin, 1.0f - motorMixMax);

		for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
		{
			float motorOutput = motorMix[i] + throttle;
			motorOutput = 158.0f + 1889.0f * motorOutput;
			motorOutput = constrain(motorOutput, 158, 2047);

			motorValues[i] = motorOutput;
		}
	}
	else
	{
		for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
		{
			motorValues[i] = 0;
		}
	}

	motorsVtable.writeMotors(motorValues);
}

void stopMotors(void)
{
	for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
	{
		motorValues[i] = 0;
	}

	motorsVtable.writeMotors(motorValues);

	// give the timers and ESCs a chance to react.
	delay(50);
}
