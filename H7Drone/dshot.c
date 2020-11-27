#include "dshot.h"

#include <stm32h7xx_ll_dma.h>
#include <stm32h7xx_ll_tim.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_hal_rcc.h>

__attribute__((section(".DMA_RAM"))) uint32_t dshotOutputBuffer[DSHOT_DMA_BUFFER_SIZE];

uint16_t prepareDshotPacket(uint16_t value, bool requestTelemetry);
static uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);

void dshotWrite(timeUs_t currentTimeUs)
{
	uint16_t packet1 = prepareDshotPacket((1123 & 0b11111111111), false);
	uint8_t bufferSize1 = loadDmaBufferDshot((uint32_t *)dshotOutputBuffer, 1, packet1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, bufferSize1);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
	LL_TIM_SetCounter(TIM3, 0);
	LL_TIM_EnableDMAReq_CC4(TIM3);
}

void DMA1_Stream2_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC2(DMA1))
	{
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
		LL_TIM_DisableDMAReq_CC4(TIM3);
		LL_DMA_ClearFlag_TC2(DMA1);
	}
}

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

void pwmDshotMotorHardwareConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// S2 - PB1
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	__HAL_RCC_TIM3_CLK_ENABLE();
	LL_TIM_DisableCounter(TIM3);

	LL_TIM_InitTypeDef TIM_TimeBaseStructure;
	LL_TIM_StructInit(&TIM_TimeBaseStructure);

	uint32_t clockTime = timerClock(TIM3);
	uint32_t outputFreq = (600 * 1000) * 3;

	TIM_TimeBaseStructure.Prescaler = (uint16_t)(lrintf((float) clockTime / (12 * 1000000) + 0.01f) - 1);
	TIM_TimeBaseStructure.Autoreload = MOTOR_BITLENGTH;
	TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_TimeBaseStructure.RepetitionCounter = 0;
	TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;

	LL_TIM_Init(TIM3, &TIM_TimeBaseStructure);

	LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
  
	LL_TIM_OC_StructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OCInitStructure.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
	TIM_OCInitStructure.OCPolarity =  LL_TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.CompareValue = 0;

	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OCInitStructure);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_EnableARRPreload(TIM3);
	LL_TIM_EnableCounter(TIM3);

	//---DMA-------------------------------------

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_2);

	LL_DMA_InitTypeDef DMA_InitStructure;

	__HAL_RCC_DMA1_CLK_ENABLE();
  
	NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

	//-DMA TIM3 CCR1-----------

	LL_DMA_StructInit(&DMA_InitStructure);

	DMA_InitStructure.PeriphRequest = LL_DMAMUX1_REQ_TIM3_CH4;
	DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)dshotOutputBuffer;
	DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
	DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
	DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
	DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;

	DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM3->CCR4;
	DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
	DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
	DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
	DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
	DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;

	LL_DMA_Init(DMA1, LL_DMA_STREAM_2, &DMA_InitStructure);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_2);
}

uint16_t prepareDshotPacket(uint16_t value, bool requestTelemetry)
{
	uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
	requestTelemetry = false;      // reset telemetry request to make sure it's triggered only once in a row

	// compute checksum
	int csum = 0;
	int csum_data = packet;
	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data;     // xor data by nibbles
		csum_data >>= 4;
	}
	csum &= 0xf;
	// append checksum
	packet = (packet << 4) | csum;

	return packet;
}

static uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
	for (int i = 0; i < 16; i++)
	{
		dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;    // MSB first
		packet <<= 1;
	}

	return DSHOT_DMA_BUFFER_SIZE;
}
