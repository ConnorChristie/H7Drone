#include "sbus.h"
#include "system.h"
#include "serial_uart.h"
#include "control.h"
#include "rc.h"

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define SBUS_FLAG_CHANNEL_17 (1 << 0)
#define SBUS_FLAG_CHANNEL_18 (1 << 1)

sbusFrameData_t sbusFrameData;

u8 sbusFrameStatus()
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

void sbusReadRawRC(float *channelData)
{
	static u16 sbusChannelData[SBUS_CHANNEL_DATA_LENGTH];
	const sbusChannels_t *channels = &sbusFrameData.frame.frame.channels;

	sbusChannelData[0] = channels->chan0;
	sbusChannelData[1] = channels->chan1;
	sbusChannelData[2] = channels->chan2;
	sbusChannelData[3] = channels->chan3;
	sbusChannelData[4] = channels->chan4;
	sbusChannelData[5] = channels->chan5;
	sbusChannelData[6] = channels->chan6;
	sbusChannelData[7] = channels->chan7;
	sbusChannelData[8] = channels->chan8;
	sbusChannelData[9] = channels->chan9;
	sbusChannelData[10] = channels->chan10;
	sbusChannelData[11] = channels->chan11;
	sbusChannelData[12] = channels->chan12;
	sbusChannelData[13] = channels->chan13;
	sbusChannelData[14] = channels->chan14;
	sbusChannelData[15] = channels->chan15;

	if (channels->flags & SBUS_FLAG_CHANNEL_17)
	{
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
	}
	else
	{
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
	}

	if (channels->flags & SBUS_FLAG_CHANNEL_18)
	{
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
	}
	else
	{
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
	}

	for (int i = 0; i < SBUS_CHANNEL_DATA_LENGTH; i++)
	{
		channelData[i] = (5 * sbusChannelData[i] / 8) + 880;
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
