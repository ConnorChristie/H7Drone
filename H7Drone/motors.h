#pragma once

#include "platform.h"
#include "dma.h"

#include <stm32h7xx_ll_tim.h>
#include <stm32h7xx_ll_dma.h>

#define MAX_SUPPORTED_MOTORS 4

typedef struct
{
	TIM_TypeDef *timer;
	u16 outputPeriod;
	u16 timerDmaSources;
} motorDmaTimer_t;

typedef struct
{
	GPIO_TypeDef *pinPack;
	u32 pin;
	u32 alternateFunction;

	TIM_TypeDef *instance;
	u8 channel;
	u32 llChannel;
} timInstance_t;

typedef struct
{
	timInstance_t timer;
	motorDmaTimer_t *dmaTimer;
	dmaIdentifier_e dma;
	u32 dmaChannel;
	u16 timerDmaSource;
} motorInstance_t;

typedef struct
{
	void(*init)(u8 id, motorInstance_t *motor);
	void(*writeMotors)(u16 *values);
} motorsVtable_t;

extern motorInstance_t motors[MAX_SUPPORTED_MOTORS];

void motorsUpdate(timeUs_t currentTimeUs);
