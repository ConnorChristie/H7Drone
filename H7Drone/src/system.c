#include "platform.h"
#include "system.h"
#include "nvic.h"

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;
static uint32_t cpuClockFrequency = 0;

void cycleCounterInit(void)
{
#if defined(USE_HAL_DRIVER)
	cpuClockFrequency = HAL_RCC_GetSysClockFreq();
#else
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	cpuClockFrequency = clocks.SYSCLK_Frequency;
#endif
	usTicks = cpuClockFrequency / 1000000;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(STM32F7) || defined(STM32H7)
	DWT->LAR = DWT_LAR_UNLOCK_VALUE;
#elif defined(STM32F3) || defined(STM32F4)
	// Note: DWT_Type does not contain LAR member.
#define DWT_LAR
	__O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
	*(DWTLAR) = 0xC5ACCE55;
#endif

	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static volatile int sysTickPending = 0;

void SysTick_Handler(void)
{
	ATOMIC_BLOCK(NVIC_PRIO_MAX) {
		sysTickUptime++;
		sysTickValStamp = SysTick->VAL;
		sysTickPending = 0;
		(void)(SysTick->CTRL);
	}
#ifdef USE_HAL_DRIVER
	// used by the HAL for some timekeeping and timeouts, should always be 1ms
	HAL_IncTick();
#endif
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t microsISR(void)
{
	register uint32_t ms, pending, cycle_cnt;

	ATOMIC_BLOCK(NVIC_PRIO_MAX)
	{
		cycle_cnt = SysTick->VAL;

		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
		{
			// Update pending.
			// Record it for multiple calls within the same rollover period
			// (Will be cleared when serviced).
			// Note that multiple rollovers are not considered.

			sysTickPending = 1;

			// Read VAL again to ensure the value is read after the rollover.

			cycle_cnt = SysTick->VAL;
		}

		ms = sysTickUptime;
		pending = sysTickPending;
	}

	return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;

	// Call microsISR() in interrupt and elevated (non-zero) BASEPRI context

	if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI()))
	{
		return microsISR();
	}

	do
	{
		ms = sysTickUptime;
		cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayMicroseconds(uint32_t us)
{
	uint32_t startUs = micros();

	while (micros() - startUs < us) ;
}

void delay(uint32_t ms)
{
	while (ms--)
	{
		delayMicroseconds(1000);
	}
}

interruptContext_t irqContexts[7];

static const uint8_t extiGroupIRQn[7] = {
	EXTI0_IRQn,
	EXTI1_IRQn,
	EXTI2_IRQn,
	EXTI3_IRQn,
	EXTI4_IRQn,
	EXTI9_5_IRQn,
	EXTI15_10_IRQn
};

void enableInterrupt(uint8_t group, interruptFuncPtr fn, void *ctx)
{
	irqContexts[group].callback = fn;
	irqContexts[group].data = ctx;

	HAL_NVIC_SetPriority(extiGroupIRQn[group], 0x0f, 0x0f);
	HAL_NVIC_EnableIRQ(extiGroupIRQn[group]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int idx = GPIOPinIdx(GPIO_Pin);

	irqContexts[idx].callback(irqContexts[idx].data);
}

void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

// zero based pin index
int GPIOPinIdx(uint16_t pin)
{
	return 31 - __builtin_clz(pin);  // CLZ is a bit faster than FFS
}
