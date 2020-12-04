#include "tasks.h"
#include "imu.h"
#include "flight.h"
#include "dshot.h"

#include <stm32h7xx_hal.h>
#include <stdio.h>

void taskGyroSample(timeUs_t currentTimeUs)
{
	imuUpdateGyroReadings();
}

void taskFiltering(timeUs_t currentTimeUs)
{
	imuFilterGyro(currentTimeUs);
}

void taskAccelSample(timeUs_t currentTimeUs)
{
	imuUpdateAccelReadings();
}

void taskMagSample(timeUs_t currentTimeUs)
{
	imuUpdateMagReadings();
}

bool ledStatus = false;

void taskLed(timeUs_t currentTimeUs)
{
	if (ledStatus)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	}

	ledStatus = !ledStatus;
}

task_t tasks[TASK_COUNT] = { 
	[TASK_SYSTEM] = DEFINE_TASK("SYSTEM", "LOAD", NULL, taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
	[TASK_LED] = DEFINE_TASK("LED", NULL, NULL, taskLed, TASK_PERIOD_MS(500), TASK_PRIORITY_LOW),
	[TASK_GYRO] = DEFINE_TASK("GYRO", NULL, NULL, taskGyroSample, TASK_PERIOD_US(125), TASK_PRIORITY_REALTIME),
	[TASK_FILTER] = DEFINE_TASK("FILTER", NULL, NULL, taskFiltering, TASK_PERIOD_US(125), TASK_PRIORITY_REALTIME),
	[TASK_ACCEL] = DEFINE_TASK("ACCEL", NULL, NULL, taskAccelSample, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
	[TASK_COMPASS] = DEFINE_TASK("COMPASS", NULL, NULL, taskMagSample, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW),
	[TASK_FLIGHT] = DEFINE_TASK("FLIGHT", NULL, NULL, flightUpdate, TASK_PERIOD_US(10000), TASK_PRIORITY_HIGH),
	[TASK_DSHOT] = DEFINE_TASK("DSHOT", NULL, NULL, dshotWrite, TASK_PERIOD_US(100), TASK_PRIORITY_REALTIME)
};

task_t* getTask(uint8_t taskId)
{
	return &tasks[taskId];
}
