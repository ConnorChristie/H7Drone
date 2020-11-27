#pragma once

#include <stdlib.h>
#include "tasks.h"

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       19

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

extern uint32_t dshotOutputBuffer[DSHOT_DMA_BUFFER_SIZE];

void pwmDshotMotorHardwareConfig(void);
void dshotWrite(timeUs_t currentTimeUs);
