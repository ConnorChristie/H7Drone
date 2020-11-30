#include "dshot.h"
#include "timer.h"
#include <string.h>

__attribute__((section(".DMA_RAM"))) u32 dshotOutputBuffer[4][DSHOT_DMA_BUFFER_SIZE];

motorInstance_t motors[4];

u16 prepareDshotPacket(u16 value);
static u8 loadDmaBufferDshot(u32 *dmaBuffer, int stride, u16 packet);

void dshotWrite(timeUs_t currentTimeUs)
{
	for (u8 i = 0; i < 1; i++)
	{
		motorInstance_t *motor = &motors[i];

		u16 packet = prepareDshotPacket(1000);
		u8 bufferSize = loadDmaBufferDshot((u32 *)&dshotOutputBuffer[i][0], 1, packet);

		LL_DMA_SetDataLength(motor->dma.instance, motor->dma.stream, bufferSize);
		LL_DMA_EnableStream(motor->dma.instance, motor->dma.stream);

		LL_TIM_DisableARRPreload(motor->timer.instance);
		motor->timer.instance->ARR = MOTOR_BITLENGTH - 1;

		LL_TIM_SetCounter(motor->timer.instance, 0);

		timerSetDMAReq(motor->timer.instance, motor->timer.channel, ENABLE);
	}
}

void DMA1_Stream2_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC2(DMA1))
	{
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
		timerSetDMAReq(TIM3, TIM_CHANNEL_4, DISABLE);
		LL_DMA_ClearFlag_TC2(DMA1);
	}
}

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

void dshotInit(u8 id, motorInstance_t motor)
{
	motor.timerDmaSource = timerDmaSource(motor.timer.channel);

	switch (motor.timer.channel) {
	case TIM_CHANNEL_1: motor.timer.llChannel = LL_TIM_CHANNEL_CH1; break;
	case TIM_CHANNEL_2: motor.timer.llChannel = LL_TIM_CHANNEL_CH2; break;
	case TIM_CHANNEL_3: motor.timer.llChannel = LL_TIM_CHANNEL_CH3; break;
	case TIM_CHANNEL_4: motor.timer.llChannel = LL_TIM_CHANNEL_CH4; break;
	}

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = motor.timer.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = motor.timer.alternateFunction;
	HAL_GPIO_Init(motor.timer.pinPack, &GPIO_InitStruct);

	RCC_ClockCmd(timerRCC(motor.timer.instance), ENABLE);
	LL_TIM_DisableCounter(motor.timer.instance);
	LL_TIM_DeInit(motor.timer.instance);

	{
		LL_TIM_InitTypeDef TIM_TimeBaseStructure;
		LL_TIM_StructInit(&TIM_TimeBaseStructure);

		TIM_TimeBaseStructure.Prescaler = (u16)(lrintf((float) timerClock(motor.timer.instance) / (12 * 1000000) + 0.01f) - 1);
		TIM_TimeBaseStructure.Autoreload = MOTOR_BITLENGTH - 1;
		TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
		TIM_TimeBaseStructure.RepetitionCounter = 0;
		TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
		LL_TIM_Init(motor.timer.instance, &TIM_TimeBaseStructure);
	}

	LL_DMA_DisableStream(motor.dma.instance, motor.dma.stream);
	LL_DMA_DeInit(motor.dma.instance, motor.dma.stream);

	{
		RCC_ClockCmd(motor.dma.instance == DMA1 ? RCC_AHB1(DMA1) : RCC_AHB1(DMA2), ENABLE);

		LL_DMA_InitTypeDef DMA_InitStructure;
		LL_DMA_StructInit(&DMA_InitStructure);

		DMA_InitStructure.PeriphRequest = motor.dma.channel;
		DMA_InitStructure.MemoryOrM2MDstAddress = (u32)&dshotOutputBuffer[id][0];
		DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
		DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
		DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
		DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
		DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;

		DMA_InitStructure.PeriphOrM2MSrcAddress = (u32)(volatile u32*)((volatile char*)&motor.timer.instance->CCR1 + motor.timer.channel);
		DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
		DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
		DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
		DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
		DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
		DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
		DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;

		NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 1));
		NVIC_EnableIRQ(DMA1_Stream2_IRQn);

		LL_DMA_Init(motor.dma.instance, motor.dma.stream, &DMA_InitStructure);
		LL_DMA_EnableIT_TC(motor.dma.instance, motor.dma.stream);
	}

	LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
	LL_TIM_OC_StructInit(&TIM_OCInitStructure);

	TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OCInitStructure.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
	TIM_OCInitStructure.OCPolarity =  LL_TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.CompareValue = 0;

	LL_TIM_OC_Init(motor.timer.instance, motor.timer.llChannel, &TIM_OCInitStructure);
	LL_TIM_OC_EnablePreload(motor.timer.instance, motor.timer.llChannel);
	LL_TIM_OC_DisableFast(motor.timer.instance, motor.timer.llChannel);

	LL_TIM_CC_EnableChannel(motor.timer.instance, motor.timer.llChannel);

	LL_TIM_EnableAllOutputs(motor.timer.instance);
	LL_TIM_EnableARRPreload(motor.timer.instance);
	LL_TIM_EnableCounter(motor.timer.instance);

	motors[id] = motor;
}

u16 prepareDshotPacket(u16 value)
{
	u16 packet = value << 1;

	// compute checksum
	int csum = 0;
	int csum_data = packet;
	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data;     // xor data by nibbles
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
	for (i = 0; i < 16; i++) {
		dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;   // MSB first
		packet <<= 1;
	}

	dmaBuffer[i++ * stride] = 0;
	dmaBuffer[i++ * stride] = 0;

	return DSHOT_DMA_BUFFER_SIZE;
}
