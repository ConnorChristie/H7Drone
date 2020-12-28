#pragma once

#include "platform.h"
#include "motors.h"

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

void dshotInit(u8 id, motorInstance_t *motor);
void dshotWriteMotors(u16 *values);
