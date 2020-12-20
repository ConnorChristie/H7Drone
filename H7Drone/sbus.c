#include "sbus.h"
#include "system.h"

static void sbusDataReceive(u16 c, void *data)
{
	sbusFrameData_t *sbusFrameData = data;

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

void sbusInit(rxConfig_t *rxConfig)
{
	
}
