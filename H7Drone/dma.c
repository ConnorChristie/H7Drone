#include "dma.h"
#include "rcc.h"

static dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
	DEFINE_DMA_CHANNEL(DMA1, 0, 0),
	DEFINE_DMA_CHANNEL(DMA1, 1, 6),
	DEFINE_DMA_CHANNEL(DMA1, 2, 16),
	DEFINE_DMA_CHANNEL(DMA1, 3, 22),
	DEFINE_DMA_CHANNEL(DMA1, 4, 32),
	DEFINE_DMA_CHANNEL(DMA1, 5, 38),
	DEFINE_DMA_CHANNEL(DMA1, 6, 48),
	DEFINE_DMA_CHANNEL(DMA1, 7, 54),

	DEFINE_DMA_CHANNEL(DMA2, 0, 0),
	DEFINE_DMA_CHANNEL(DMA2, 1, 6),
	DEFINE_DMA_CHANNEL(DMA2, 2, 16),
	DEFINE_DMA_CHANNEL(DMA2, 3, 22),
	DEFINE_DMA_CHANNEL(DMA2, 4, 32),
	DEFINE_DMA_CHANNEL(DMA2, 5, 38),
	DEFINE_DMA_CHANNEL(DMA2, 6, 48),
	DEFINE_DMA_CHANNEL(DMA2, 7, 54),
};

DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_ST7_HANDLER)

DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_ST7_HANDLER)

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
	const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

	RCC_ClockCmd(dmaDescriptors[index].instance.dma == DMA1 ? RCC_AHB1(DMA1) : RCC_AHB1(DMA2), ENABLE);
	dmaDescriptors[index].irqHandlerCallback = callback;
	dmaDescriptors[index].userParam = userParam;

	NVIC_SetPriority(dmaDescriptors[index].irqN, priority);
	NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef* stream)
{
	for (int i = 0; i < DMA_LAST_HANDLER; i++)
	{
		if (dmaDescriptors[i].instance.ref == stream)
		{
			return i + 1;
		}
	}

	return 0;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
	return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
}
