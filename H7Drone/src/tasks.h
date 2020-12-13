#pragma once

#include "scheduler.h"

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
