#include "sbus.h"
#include "system.h"
#include "serial_uart.h"
#include "control.h"

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

sbusFrameData_t sbusFrameData;

static u8 sbusFrameStatus()
{
	if (!sbusFrameData.done)
	{
		return RX_FRAME_PENDING;
	}
	
	sbusFrameData.done = false;
	
	if (sbusFrameData.frame.frame.channels.flags & SBUS_FLAG_FAILSAFE_ACTIVE)
	{
		// internal failsafe enabled and rx failsafe flag set
		// RX *should* still be sending valid channel data (repeated), so use it.
		return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
	}

	if (sbusFrameData.frame.frame.channels.flags & SBUS_FLAG_SIGNAL_LOSS)
	{
		// The received data is a repeat of the last valid data so can be considered complete.
		return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
	}

	return RX_FRAME_COMPLETE;
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
	const u8 frameStatus = sbusFrameStatus();
	bool signalReceived = false;

	if (frameStatus & RX_FRAME_COMPLETE)
	{
		bool rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
		bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;

		return !(rxIsInFailsafeMode || rxFrameDropped);
	}

	return false;
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	setThrottle(sbusFrameData.frame.frame.channels.chan0);

	updateSetpointRate(0, sbusFrameData.frame.frame.channels.chan1);
	updateSetpointRate(1, sbusFrameData.frame.frame.channels.chan2);
	updateSetpointRate(2, sbusFrameData.frame.frame.channels.chan3);
}

static void sbusDataReceive(u16 c, void *data)
{
	sbusFrameData_t *sbusFrameData = (sbusFrameData_t *)data;

	const timeUs_t nowUs = microsISR();
	const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

	if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500))
	{
		sbusFrameData->position = 0;
	}

	if (sbusFrameData->position == 0)
	{
		if (c != SBUS_FRAME_BEGIN_BYTE)
		{
			return;
		}
		sbusFrameData->startAtUs = nowUs;
	}

	if (sbusFrameData->position < SBUS_FRAME_SIZE)
	{
		sbusFrameData->frame.bytes[sbusFrameData->position++] = (u8)c;
		if (sbusFrameData->position < SBUS_FRAME_SIZE)
		{
			sbusFrameData->done = false;
		}
		else
		{
			sbusFrameData->done = true;
		}
	}
}

void sbusInit()
{
	serialPort_s port = { };
	
	port.init.BaudRate = 100000;
	port.init.Mode = UART_MODE_RX;
	port.init.Parity = UART_PARITY_EVEN;
	port.init.StopBits = USART_STOPBITS_2;
	port.init.WordLength = UART_WORDLENGTH_9B;
	port.init.HwFlowCtl = UART_HWCONTROL_NONE;
	port.init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

	port.advInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
	port.advInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

	port.rxCallback = sbusDataReceive;
	port.rxCallbackData = &sbusFrameData;

	uartInit(UARTDEV_6, port);
}
