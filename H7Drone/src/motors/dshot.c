#include "dshot.h"
#include "system.h"
#include "timer.h"
#include "dma.h"
#include "nvic.h"

DMA_RAM u32 dshotOutputBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_SIZE];

u16 prepareDshotPacket(u16 value);
static u8 loadDmaBufferDshot(u32 *dmaBuffer, int stride, u16 packet);

static u16 cnt = 0;
timeUs_t lastTime = 0;

FAST_DATA_ZERO_INIT u8 dmaMotorTimerCount = 0;
FAST_DATA_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_SUPPORTED_MOTORS];

bool isDmaTimerConfigured(TIM_TypeDef *timer);
motorDmaTimer_t* getDmaTimer(TIM_TypeDef *timer);

FAST_CODE void dshotWriteInt(u8 id, u16 value)
{
	const motorInstance_t *motor = &motors[id];
	const dmaChannelDescriptor_t *dma = dmaGetDescriptorByIdentifier(motor->dma);

	u16 packet = prepareDshotPacket(value);
	u8 bufferSize = loadDmaBufferDshot((u32 *)&dshotOutputBuffer[id][0], 1, packet);

	LL_DMA_SetDataLength(dma->instance.dma, dma->instance.stream, bufferSize);
	LL_DMA_EnableStream(dma->instance.dma, dma->instance.stream);

	motor->dmaTimer->timerDmaSources |= motor->timerDmaSource;
}

FAST_CODE void dshotUpdateComplete()
{
	for (u8 i = 0; i < dmaMotorTimerCount; i++)
	{
		const motorDmaTimer_t *dmaTimer = &dmaMotorTimers[i];

		LL_TIM_DisableARRPreload(dmaTimer->timer);
		dmaTimer->timer->ARR = dmaTimer->outputPeriod;

		LL_TIM_SetCounter(dmaTimer->timer, 0);
		timerSetDMAReqStatus(dmaTimer->timer, dmaTimer->timerDmaSources, ENABLE);
	}
}

FAST_CODE void dshotWriteMotors(u16 *values)
{
	for (u8 i = 0; i < MAX_SUPPORTED_MOTORS; i++)
	{
		dshotWriteInt(i, values[i]);
	}

	dshotUpdateComplete();
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
	if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
	{
		LL_DMA_DisableStream(descriptor->instance.dma, descriptor->instance.stream);

		const motorInstance_t *motor = &motors[descriptor->userParam];
		timerSetDMAReqStatus(motor->timer.instance, motor->timerDmaSource, DISABLE);

		motor->dmaTimer->timerDmaSources &= ~motor->timerDmaSource;

		DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
	}
}

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

void dshotInit(u8 id, motorInstance_t *motor)
{
	dmaChannelDescriptor_t *dma = dmaGetDescriptorByIdentifier(motor->dma);

	switch (motor->timer.channel) {
	case TIM_CHANNEL_1: motor->timer.llChannel = LL_TIM_CHANNEL_CH1; break;
	case TIM_CHANNEL_2: motor->timer.llChannel = LL_TIM_CHANNEL_CH2; break;
	case TIM_CHANNEL_3: motor->timer.llChannel = LL_TIM_CHANNEL_CH3; break;
	case TIM_CHANNEL_4: motor->timer.llChannel = LL_TIM_CHANNEL_CH4; break;
	}

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = motor->timer.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = motor->timer.alternateFunction;
	HAL_GPIO_Init(motor->timer.pinPack, &GPIO_InitStruct);

	if (!isDmaTimerConfigured(motor->timer.instance))
	{
		RCC_ClockCmd(timerRCC(motor->timer.instance), ENABLE);
		LL_TIM_DisableCounter(motor->timer.instance);
		LL_TIM_DeInit(motor->timer.instance);

		LL_TIM_InitTypeDef TIM_TimeBaseStructure;
		LL_TIM_StructInit(&TIM_TimeBaseStructure);

		TIM_TimeBaseStructure.Prescaler = (u16)(lrintf((float) timerClock(motor->timer.instance) / MOTOR_DSHOT600_HZ + 0.01f) - 1);
		TIM_TimeBaseStructure.Autoreload = MOTOR_BITLENGTH - 1;
		TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
		TIM_TimeBaseStructure.RepetitionCounter = 0;
		TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
		LL_TIM_Init(motor->timer.instance, &TIM_TimeBaseStructure);
	}

	LL_DMA_DisableStream(dma->instance.dma, dma->instance.stream);
	LL_DMA_DeInit(dma->instance.dma, dma->instance.stream);

	{
		LL_DMA_InitTypeDef DMA_InitStructure;
		LL_DMA_StructInit(&DMA_InitStructure);

		DMA_InitStructure.PeriphRequest = motor->dmaChannel;
		DMA_InitStructure.MemoryOrM2MDstAddress = (u32)&dshotOutputBuffer[id][0];
		DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
		DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
		DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
		DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
		DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;

		DMA_InitStructure.PeriphOrM2MSrcAddress = (u32)((volatile char*)&motor->timer.instance->CCR1 + motor->timer.channel);
		DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
		DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
		DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
		DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
		DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
		DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
		DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;

		dmaSetHandler(motor->dma, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, id);

		LL_DMA_Init(dma->instance.dma, dma->instance.stream, &DMA_InitStructure);
		LL_DMA_EnableIT_TC(dma->instance.dma, dma->instance.stream);
	}

	LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
	LL_TIM_OC_StructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OCInitStructure.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
	TIM_OCInitStructure.OCPolarity =  LL_TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.CompareValue = 0;

	LL_TIM_OC_Init(motor->timer.instance, motor->timer.llChannel, &TIM_OCInitStructure);
	LL_TIM_OC_EnablePreload(motor->timer.instance, motor->timer.llChannel);
	LL_TIM_OC_DisableFast(motor->timer.instance, motor->timer.llChannel);

	LL_TIM_CC_EnableChannel(motor->timer.instance, motor->timer.llChannel);

	LL_TIM_EnableAllOutputs(motor->timer.instance);
	LL_TIM_EnableARRPreload(motor->timer.instance);
	LL_TIM_EnableCounter(motor->timer.instance);

	motor->timerDmaSource = timerDmaSource(motor->timer.channel);
	motor->dmaTimer = getDmaTimer(motor->timer.instance);
	motor->dmaTimer->outputPeriod = MOTOR_BITLENGTH - 1;
}

u16 prepareDshotPacket(u16 value)
{
	u16 packet = value << 1;

	// compute checksum
	int csum = 0;
	int csum_data = packet;
	for (int i = 0; i < 3; i++)
	{
		// xor data by nibbles
		csum ^=  csum_data;
		csum_data >>= 4;
	}

	// append checksum
	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

static u8 loadDmaBufferDshot(u32 *dmaBuffer, int stride, u16 packet)
{
	int i;
	for (i = 0; i < 16; i++)
	{
		// MSB first
		dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	dmaBuffer[i++ * stride] = 0;
	dmaBuffer[i++ * stride] = 0;

	return DSHOT_DMA_BUFFER_SIZE;
}

bool isDmaTimerConfigured(TIM_TypeDef *timer)
{
	for (u8 i = 0; i < dmaMotorTimerCount; i++)
	{
		if (dmaMotorTimers[i].timer == timer)
		{
			return true;
		}
	}
	return false;
}

motorDmaTimer_t* getDmaTimer(TIM_TypeDef *timer)
{
	for (u8 i = 0; i < dmaMotorTimerCount; i++)
	{
		if (dmaMotorTimers[i].timer == timer)
		{
			return &dmaMotorTimers[i];
		}
	}
	dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
	return &dmaMotorTimers[dmaMotorTimerCount - 1];
}
