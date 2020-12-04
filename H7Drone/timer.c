#include "timer.h"
#include <stm32h7xx_ll_tim.h>

#ifdef STM32H7
const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
	{ .TIMx = TIM1, .rcc = RCC_APB2(TIM1), .inputIrq = TIM1_CC_IRQn },
	{ .TIMx = TIM2, .rcc = RCC_APB1L(TIM2), .inputIrq = TIM2_IRQn },
	{ .TIMx = TIM3, .rcc = RCC_APB1L(TIM3), .inputIrq = TIM3_IRQn },
	{ .TIMx = TIM4, .rcc = RCC_APB1L(TIM4), .inputIrq = TIM4_IRQn },
	{ .TIMx = TIM5, .rcc = RCC_APB1L(TIM5), .inputIrq = TIM5_IRQn },
	{ .TIMx = TIM6, .rcc = RCC_APB1L(TIM6), .inputIrq = 0 },
	{ .TIMx = TIM7, .rcc = RCC_APB1L(TIM7), .inputIrq = 0 },
	{ .TIMx = TIM8, .rcc = RCC_APB2(TIM8), .inputIrq = TIM8_CC_IRQn },
	{ .TIMx = TIM12, .rcc = RCC_APB1L(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn },
	{ .TIMx = TIM13, .rcc = RCC_APB1L(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn },
	{ .TIMx = TIM14, .rcc = RCC_APB1L(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn },
	{ .TIMx = TIM15, .rcc = RCC_APB2(TIM15), .inputIrq = TIM15_IRQn },
	{ .TIMx = TIM16, .rcc = RCC_APB2(TIM16), .inputIrq = TIM16_IRQn },
	{ .TIMx = TIM17, .rcc = RCC_APB2(TIM17), .inputIrq = TIM17_IRQn },
};
#endif

rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
	for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++)
	{
		if (timerDefinitions[i].TIMx == tim)
		{
			return timerDefinitions[i].rcc;
		}
	}
	return 0;
}

u32 timerClock(TIM_TypeDef *tim)
{
	int timpre;
	u32 pclk;
	u32 ppre;

	// Implement the table:
	// RM0433 "Table 48. Ratio between clock timer and pclk"

	if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17)
	{
		// Timers on APB2
		pclk = HAL_RCC_GetPCLK2Freq();
		ppre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos;
	}
	else
	{
		// Timers on APB1
		pclk = HAL_RCC_GetPCLK1Freq();
		ppre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
	}

	timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) ? 1 : 0;

	int index = (timpre << 3) | ppre;

	static uint8_t periphToKernel[16] = { // The mutiplier table
		1, 1, 1, 1, 2, 2, 2, 2, // TIMPRE = 0
		1, 1, 1, 1, 2, 4, 4, 4  // TIMPRE = 1
	};

	return pclk * periphToKernel[index];
}

u16 timerDmaSource(u8 channel)
{
	switch (channel) {
	case TIM_CHANNEL_1:
		return TIM_DMA_CC1;
	case TIM_CHANNEL_2:
		return TIM_DMA_CC2;
	case TIM_CHANNEL_3:
		return TIM_DMA_CC3;
	case TIM_CHANNEL_4:
		return TIM_DMA_CC4;
	}
	return 0;
}

#define LL_TIM_IT(tim, cc, state) \
	if (state == ENABLE) \
		LL_TIM_EnableDMAReq_ ## cc(tim); \
	else \
		LL_TIM_DisableDMAReq_ ## cc(tim);

void timerSetDMAReqStatus(TIM_TypeDef *tim, u8 channel, FunctionalState state)
{
	switch (channel) {
	case TIM_CHANNEL_1:
		LL_TIM_IT(tim, CC1, state);
		break;
	case TIM_CHANNEL_2:
		LL_TIM_IT(tim, CC2, state);
		break;
	case TIM_CHANNEL_3:
		LL_TIM_IT(tim, CC3, state);
		break;
	case TIM_CHANNEL_4:
		LL_TIM_IT(tim, CC4, state);
		break;
	}
}
