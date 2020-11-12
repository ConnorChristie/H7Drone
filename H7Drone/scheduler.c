#include "scheduler.h"
#include <stm32h7xx_hal.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "system.h"
#include "tasks.h"

#define TASK_AVERAGE_EXECUTE_FALLBACK_US 30 // Default task average time if USE_TASK_STATISTICS is not defined
#define TASK_AVERAGE_EXECUTE_PADDING_US 5   // Add a little padding to the average execution time

// DEBUG_SCHEDULER, timings for:
// 0 - gyroUpdate()
// 1 - pidController()
// 2 - time spent in scheduler
// 3 - time spent executing check function

static task_t *currentTask = NULL;

static bool calculateTaskStatistics;

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
uint16_t averageSystemLoadPercent = 0;

int taskQueueSize = 0;

static int periodCalculationBasisOffset = offsetof(task_t, lastExecutedAtUs);

// No need for a linked list for the queue, since items are only inserted at startup

task_t* taskQueueArray[TASK_COUNT + 1];   // extra item for NULL pointer at end of queue

void queueClear(void)
{
	memset(taskQueueArray, 0, sizeof(taskQueueArray));
	taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
	for (int ii = 0; ii < taskQueueSize; ++ii)
	{
		if (taskQueueArray[ii] == task)
		{
			return true;
		}
	}
	return false;
}

bool queueAdd(task_t *task)
{
	if ((taskQueueSize >= TASK_COUNT) || queueContains(task))
	{
		return false;
	}
	for (int ii = 0; ii <= taskQueueSize; ++ii)
	{
		if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority)
		{
			memmove(&taskQueueArray[ii + 1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
			taskQueueArray[ii] = task;
			++taskQueueSize;
			return true;
		}
	}
	return false;
}

bool queueRemove(task_t *task)
{
	for (int ii = 0; ii < taskQueueSize; ++ii)
	{
		if (taskQueueArray[ii] == task)
		{
			memmove(&taskQueueArray[ii], &taskQueueArray[ii + 1], sizeof(task) * (taskQueueSize - ii));
			--taskQueueSize;
			return true;
		}
	}
	return false;
}

#if defined(USE_TASK_STATISTICS)
timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
	checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
	checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
	checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
	checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}
#endif

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
	task_t* task = getTask(taskId);
	
	taskInfo->isEnabled = queueContains(task);
	taskInfo->desiredPeriodUs = task->desiredPeriodUs;
	taskInfo->staticPriority = task->staticPriority;
#if defined(USE_TASK_STATISTICS)
	taskInfo->taskName = task->taskName;
	taskInfo->subTaskName = task->subTaskName;
	taskInfo->maxExecutionTimeUs = task->maxExecutionTimeUs;
	taskInfo->totalExecutionTimeUs = task->totalExecutionTimeUs;
	taskInfo->averageExecutionTimeUs = task->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
	taskInfo->averageDeltaTimeUs = task->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
	taskInfo->latestDeltaTimeUs = task->taskLatestDeltaTimeUs;
	taskInfo->movingAverageCycleTimeUs = task->movingAverageCycleTimeUs;
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
	if (taskId == TASK_SELF)
	{
		task_t *task = currentTask;
		task->desiredPeriodUs = MAX(100, newPeriodUs);    // Limit delay to 100us (10 kHz) to prevent scheduler clogging
	}
	else if (taskId < TASK_COUNT)
	{
		getTask(taskId)->desiredPeriodUs = MAX(100, newPeriodUs);      // Limit delay to 100us (10 kHz) to prevent scheduler clogging
	}
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
	if (taskId == TASK_SELF || taskId < TASK_COUNT)
	{
		task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
		if (enabled && task->taskFunc)
		{
			queueAdd(task);
		}
		else
		{
			queueRemove(task);
		}
	}
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
	if (taskId == TASK_SELF)
	{
		return currentTask->taskLatestDeltaTimeUs;
	}
	else if (taskId < TASK_COUNT)
	{
		return getTask(taskId)->taskLatestDeltaTimeUs;
	}
	else
	{
		return 0;
	}
}

void schedulerSetCalulateTaskStatistics(bool calculateTaskStatisticsToUse)
{
	calculateTaskStatistics = calculateTaskStatisticsToUse;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
#if defined(USE_TASK_STATISTICS)
	if (taskId == TASK_SELF)
	{
		currentTask->movingSumExecutionTimeUs = 0;
		currentTask->movingSumDeltaTimeUs = 0;
		currentTask->totalExecutionTimeUs = 0;
		currentTask->maxExecutionTimeUs = 0;
	}
	else if (taskId < TASK_COUNT)
	{
		getTask(taskId)->movingSumExecutionTimeUs = 0;
		getTask(taskId)->movingSumDeltaTimeUs = 0;
		getTask(taskId)->totalExecutionTimeUs = 0;
		getTask(taskId)->maxExecutionTimeUs = 0;
	}
#else
	UNUSED(taskId);
#endif
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
#if defined(USE_TASK_STATISTICS)
	if (taskId == TASK_SELF)
	{
		currentTask->maxExecutionTimeUs = 0;
	}
	else if (taskId < TASK_COUNT)
	{
		getTask(taskId)->maxExecutionTimeUs = 0;
	}
#else
	UNUSED(taskId);
#endif
}

#if defined(USE_TASK_STATISTICS)
void schedulerResetCheckFunctionMaxExecutionTime(void)
{
	checkFuncMaxExecutionTimeUs = 0;
}
#endif

void schedulerInit(void)
{
	calculateTaskStatistics = true;
	queueClear();
	queueAdd(getTask(TASK_SYSTEM));
}

void schedulerOptimizeRate(bool optimizeRate)
{
	periodCalculationBasisOffset = optimizeRate ? offsetof(task_t, lastDesiredAt) : offsetof(task_t, lastExecutedAtUs);
}

inline static timeUs_t getPeriodCalculationBasis(const task_t* task)
{
	if (task->staticPriority == TASK_PRIORITY_REALTIME)
	{
		return *(timeUs_t*)((uint8_t*)task + periodCalculationBasisOffset);
	}
	else
	{
		return task->lastExecutedAtUs;
	}
}

timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
	timeUs_t taskExecutionTimeUs = 0;

	if (selectedTask)
	{
		currentTask = selectedTask;
		selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastExecutedAtUs);
#if defined(USE_TASK_STATISTICS)
		float period = currentTimeUs - selectedTask->lastExecutedAtUs;
#endif
		selectedTask->lastExecutedAtUs = currentTimeUs;
		selectedTask->lastDesiredAt += (cmpTimeUs(currentTimeUs, selectedTask->lastDesiredAt) / selectedTask->desiredPeriodUs) * selectedTask->desiredPeriodUs;
		selectedTask->dynamicPriority = 0;

		// Execute task
#if defined(USE_TASK_STATISTICS)
		if (calculateTaskStatistics)
		{
			const timeUs_t currentTimeBeforeTaskCallUs = micros();
			selectedTask->taskFunc(currentTimeBeforeTaskCallUs);
			taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
			selectedTask->movingSumExecutionTimeUs += taskExecutionTimeUs - selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
			selectedTask->movingSumDeltaTimeUs += selectedTask->taskLatestDeltaTimeUs - selectedTask->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
			selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;     // time consumed by scheduler + task
			selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
			selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
		}
		else
#endif
		{
			selectedTask->taskFunc(currentTimeUs);
		}
	}

	return taskExecutionTimeUs;
}

void scheduler(void)
{
	// Cache currentTime
	const timeUs_t schedulerStartTimeUs = micros();
	timeUs_t currentTimeUs = schedulerStartTimeUs;
	timeUs_t taskExecutionTimeUs = 0;
	task_t *selectedTask = NULL;
	uint16_t selectedTaskDynamicPriority = 0;
	uint16_t waitingTasks = 0;
	bool realtimeTaskRan = false;

	task_t *gyroTask = getTask(TASK_GYRO);
	timeDelta_t gyroTaskDelayUs = 0;
	if (gyroTask != NULL)
	{
		const timeUs_t gyroExecuteTimeUs = getPeriodCalculationBasis(gyroTask) + gyroTask->desiredPeriodUs;
		gyroTaskDelayUs = cmpTimeUs(gyroExecuteTimeUs, currentTimeUs);        // time until the next expected gyro sample

		if (cmpTimeUs(currentTimeUs, gyroExecuteTimeUs) >= 0)
		{
			for (uint8_t i = 0; i < taskQueueSize; i++)
			{
				task_t *task = taskQueueArray[i];
				
				if (task->staticPriority == TASK_PRIORITY_REALTIME)
				{
					taskExecutionTimeUs += schedulerExecuteTask(task, currentTimeUs);
				}
			}

			realtimeTaskRan = true;
		}
	}
	else
	{
		realtimeTaskRan = true;
	}

	if (realtimeTaskRan || (gyroTaskDelayUs > GYRO_TASK_GUARD_INTERVAL_US))
	{
		// The task to be invoked
		// Update task dynamic priorities
		for (uint8_t i = 0 ; i < taskQueueSize ; i++)
		{
			task_t *task = taskQueueArray[i];

			if (task->staticPriority != TASK_PRIORITY_REALTIME)
			{
				// Task has checkFunc - event driven
				if (task->checkFunc)
				{
					const timeUs_t currentTimeBeforeCheckFuncCallUs = currentTimeUs;
					// Increase priority for event driven tasks
					if (task->dynamicPriority > 0)
					{
						task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAtUs) / task->desiredPeriodUs);
						task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
						waitingTasks++;
					}
					else if (task->checkFunc(currentTimeBeforeCheckFuncCallUs, cmpTimeUs(currentTimeBeforeCheckFuncCallUs, task->lastExecutedAtUs)))
					{
#if defined(USE_TASK_STATISTICS)
						if (calculateTaskStatistics)
						{
							const uint32_t checkFuncExecutionTimeUs = micros() - currentTimeBeforeCheckFuncCallUs;
							checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
							checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
							checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;     // time consumed by scheduler + task
							checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
						}
#endif
						task->lastSignaledAtUs = currentTimeBeforeCheckFuncCallUs;
						task->taskAgeCycles = 1;
						task->dynamicPriority = 1 + task->staticPriority;
						waitingTasks++;
					}
					else
					{
						task->taskAgeCycles = 0;
					}
				}
				else
				{
					// Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
					// Task age is calculated from last execution
					task->taskAgeCycles = ((currentTimeUs - getPeriodCalculationBasis(task)) / task->desiredPeriodUs);
					if (task->taskAgeCycles > 0)
					{
						task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
						waitingTasks++;
					}
				}

				if (task->dynamicPriority > selectedTaskDynamicPriority)
				{
					selectedTaskDynamicPriority = task->dynamicPriority;
					selectedTask = task;
				}
			}
		}

		totalWaitingTasksSamples++;
		totalWaitingTasks += waitingTasks;

		if (selectedTask)
		{
			timeDelta_t taskRequiredTimeUs = TASK_AVERAGE_EXECUTE_FALLBACK_US;    // default average time if task statistics are not available
#if defined(USE_TASK_STATISTICS)
			if (calculateTaskStatistics)
			{
				taskRequiredTimeUs = selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT + TASK_AVERAGE_EXECUTE_PADDING_US;
			}
#endif
			// Add in the time spent so far in check functions and the scheduler logic
			taskRequiredTimeUs += cmpTimeUs(micros(), currentTimeUs);
			if (realtimeTaskRan)
			{
				taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
			}
			else
			{
				selectedTask = NULL;
			}
		}
	}
}

void taskSystemLoad(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);

	// Calculate system load
	if (totalWaitingTasksSamples > 0)
	{
		averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
		totalWaitingTasksSamples = 0;
		totalWaitingTasks = 0;
	}
}

uint16_t getAverageSystemLoadPercent(void)
{
	return averageSystemLoadPercent;
}
