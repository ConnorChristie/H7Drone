#pragma once

#include "platform.h"
#include "tasks.h"

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

#define GYRO_TASK_GUARD_INTERVAL_US 10  // Don't run any other tasks if gyro task will be run soon

#if defined(USE_TASK_STATISTICS)
#define TASK_STATS_MOVING_SUM_COUNT 32
#endif

#define LOAD_PERCENTAGE_ONE 100

typedef enum
{
	TASK_PRIORITY_REALTIME = -1,
	// Task will be run outside the scheduler logic
	TASK_PRIORITY_IDLE = 0,
	// Disables dynamic scheduling, task is executed only if no other task is active this cycle
	TASK_PRIORITY_LOW = 1,
	TASK_PRIORITY_MEDIUM = 3,
	TASK_PRIORITY_MEDIUM_HIGH = 4,
	TASK_PRIORITY_HIGH = 5,
	TASK_PRIORITY_MAX = 255
} taskPriority_e;

typedef struct
{
	timeUs_t     maxExecutionTimeUs;
	timeUs_t     totalExecutionTimeUs;
	timeUs_t     averageExecutionTimeUs;
	timeUs_t     averageDeltaTimeUs;
} cfCheckFuncInfo_t;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo);
void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo);
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId);
void schedulerSetCalulateTaskStatistics(bool calculateTaskStatistics);
void schedulerResetTaskStatistics(taskId_e taskId);
void schedulerResetTaskMaxExecutionTime(taskId_e taskId);
void schedulerResetCheckFunctionMaxExecutionTime(void);

void schedulerInit(void);
void scheduler(void);
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs);
void taskSystemLoad(timeUs_t currentTimeUs);
void schedulerOptimizeRate(bool optimizeRate);
void schedulerEnableGyro(void);
uint16_t getAverageSystemLoadPercent(void);

#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
