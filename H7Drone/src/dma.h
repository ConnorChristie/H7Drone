#pragma once

#include "platform.h"

struct dmaChannelDescriptor_s;
typedef void(*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct
{
	DMA_Stream_TypeDef *ref;
	DMA_TypeDef *dma;
	u32 stream;
} dmaInstance_t;

typedef struct dmaChannelDescriptor_s
{
	dmaInstance_t               instance;
	dmaCallbackHandlerFuncPtr   irqHandlerCallback;
	uint8_t                     flagsShift;
	IRQn_Type                   irqN;
	uint32_t                    userParam;
	uint8_t                     resourceIndex;
	uint32_t                    completeFlag;
} dmaChannelDescriptor_t;

typedef enum
{
	DMA_NONE         = 0,
	DMA1_ST0_HANDLER = 1,
	DMA1_ST1_HANDLER,
	DMA1_ST2_HANDLER,
	DMA1_ST3_HANDLER,
	DMA1_ST4_HANDLER,
	DMA1_ST5_HANDLER,
	DMA1_ST6_HANDLER,
	DMA1_ST7_HANDLER,
	DMA2_ST0_HANDLER,
	DMA2_ST1_HANDLER,
	DMA2_ST2_HANDLER,
	DMA2_ST3_HANDLER,
	DMA2_ST4_HANDLER,
	DMA2_ST5_HANDLER,
	DMA2_ST6_HANDLER,
	DMA2_ST7_HANDLER,
	DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

#define DEFINE_DMA_CHANNEL(d, s, f) { \
		.instance = { \
			.ref = d ## _Stream ## s, \
			.dma = d, \
			.stream = s, \
		}, \
		.irqHandlerCallback = NULL, \
		.flagsShift = f, \
		.irqN = d ## _Stream ## s ## _IRQn, \
		.userParam = 0 \
	}

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                            const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                            dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                            if (handler) \
                                                handler(&dmaDescriptors[index]); \
                                        }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->instance.dma->HIFCR = (flag << (d->flagsShift - 32)); else d->instance.dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->instance.dma->HISR & (flag << (d->flagsShift - 32)): d->instance.dma->LISR & (flag << d->flagsShift))

#define DMA_IT_TCIF ((uint32_t)0x00000020)

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);
dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef* stream);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);
