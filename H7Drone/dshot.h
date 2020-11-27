#pragma once

#include <stm32h7xx_ll_tim.h>
#include <stm32h7xx_ll_dma.h>

#include "tasks.h"
#include "platform.h"

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       19

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

typedef struct
{
	GPIO_TypeDef *pinPack;
	u32 pin;
	u32 alternateFunction;

	TIM_TypeDef *instance;
	u32 channel;
} timInstance_t;

typedef struct
{
	DMA_TypeDef *instance;
	u32 stream;
} dmaInstance_t;

typedef struct
{
	timInstance_t timer;
	dmaInstance_t dma;
} motorInstance_t;

extern u32 dshotOutputBuffer[4][DSHOT_DMA_BUFFER_SIZE];

void dshotInit(u8 id, motorInstance_t motor);
void dshotWrite(timeUs_t currentTimeUs);
