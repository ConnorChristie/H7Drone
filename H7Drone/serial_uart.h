#pragma once

#include "platform.h"
#include "rcc.h"

typedef void(*serialReceiveCallbackPtr)(uint16_t data, void *rxCallbackData);

typedef enum
{
	UARTDEV_1,
	UARTDEV_2,
	UARTDEV_3,
	UARTDEV_4,
	UARTDEV_5,
	UARTDEV_6,
	UARTDEV_7,
	UARTDEV_8,
	UARTDEV_COUNT
} uartIdentifier_e;

typedef struct
{
	UART_InitTypeDef init;
	UART_AdvFeatureInitTypeDef advInit;

	serialReceiveCallbackPtr rxCallback;
	void *rxCallbackData;
} serialPort_s;

typedef struct
{
	USART_TypeDef *ref;
	IRQn_Type IRQn;
	rccPeriphTag_t rcc;

	u8 txPriority;
	u8 rxPriority;

	serialPort_s port;
} uartDevice_s;

void uartInit(uartIdentifier_e dev, serialPort_s port);
