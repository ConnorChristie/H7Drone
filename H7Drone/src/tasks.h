#pragma once

#include "platform.h"

typedef enum
{
	/* Actual tasks */
	TASK_SYSTEM = 0,
	TASK_LED,
	TASK_COMPASS,

	TASK_ACCEL,
	TASK_GYRO,
	TASK_FILTER,
	TASK_PID,

	TASK_FLIGHT,
	TASK_DSHOT,
	TASK_RX,
	/* Count of real tasks */
	TASK_COUNT,

	/* Service task IDs */
	TASK_NONE = TASK_COUNT,
	TASK_SELF
} taskId_e;

typedef struct
{
	const char * taskName;
	const char * subTaskName;
	bool         isEnabled;
	int8_t       staticPriority;
	timeDelta_t  desiredPeriodUs;
	timeDelta_t  latestDeltaTimeUs;
	timeUs_t     maxExecutionTimeUs;
	timeUs_t     totalExecutionTimeUs;
	timeUs_t     averageExecutionTimeUs;
	timeUs_t     averageDeltaTimeUs;
	float        movingAverageCycleTimeUs;
} taskInfo_t;

typedef struct
{
	// Configuration
#if defined(USE_TASK_STATISTICS)
	const char * taskName;
	const char * subTaskName;
#endif
	bool(*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
	void(*taskFunc)(timeUs_t currentTimeUs);
	timeDelta_t desiredPeriodUs;        // target period of execution
	const int8_t staticPriority;      // dynamicPriority grows in steps of this size

	// Scheduling
	uint16_t dynamicPriority;         // measurement of how old task was last executed, used to avoid task starvation
	uint16_t taskAgeCycles;
	timeDelta_t taskLatestDeltaTimeUs;
	timeUs_t lastExecutedAtUs;          // last time of invocation
	timeUs_t lastSignaledAtUs;          // time of invocation event for event-driven tasks
	timeUs_t lastDesiredAt;           // time of last desired execution

#if defined(USE_TASK_STATISTICS)
	// Statistics
	float    movingAverageCycleTimeUs;
	timeUs_t movingSumExecutionTimeUs;    // moving sum over 32 samples
	timeUs_t movingSumDeltaTimeUs;    // moving sum over 32 samples
	timeUs_t maxExecutionTimeUs;
	timeUs_t totalExecutionTimeUs;      // total time consumed by task since boot
#endif
} task_t;

#if defined(USE_TASK_STATISTICS)
#define DEFINE_TASK(taskNameParam, subTaskNameParam, checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .taskName = taskNameParam, \
    .subTaskName = subTaskNameParam, \
    .checkFunc = checkFuncParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
    .staticPriority = staticPriorityParam \
}
#else
#define DEFINE_TASK(taskNameParam, subTaskNameParam, checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .checkFunc = checkFuncParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
    .staticPriority = staticPriorityParam \
}
#endif

task_t* getTask(uint8_t taskId);
