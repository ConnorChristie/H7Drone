#pragma once

#include "platform.h"
#include "rcc.h"

typedef struct timerDef_s {
	TIM_TypeDef *TIMx;
	rccPeriphTag_t rcc;
	uint8_t inputIrq;
} timerDef_t;

rccPeriphTag_t timerRCC(TIM_TypeDef *tim);
u32 timerClock(TIM_TypeDef *tim);

#ifdef STM32H7
#define HARDWARE_TIMER_DEFINITION_COUNT 17
#endif

extern const timerDef_t timerDefinitions[];
