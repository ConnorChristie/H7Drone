#pragma once

#include "platform.h"
#include "scheduler.h"

#define SBUS_BAUDRATE                 100000
#define SBUS_RX_REFRESH_RATE          11000
#define SBUS_TIME_NEEDED_PER_FRAME    3000

#define SBUS_FAST_BAUDRATE              200000
#define SBUS_FAST_RX_REFRESH_RATE       6000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define SBUS_MAX_CHANNEL 18

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

typedef struct sbusChannels_s
{
	// 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
	unsigned int chan0 : 11;
	unsigned int chan1 : 11;
	unsigned int chan2 : 11;
	unsigned int chan3 : 11;
	unsigned int chan4 : 11;
	unsigned int chan5 : 11;
	unsigned int chan6 : 11;
	unsigned int chan7 : 11;
	unsigned int chan8 : 11;
	unsigned int chan9 : 11;
	unsigned int chan10 : 11;
	unsigned int chan11 : 11;
	unsigned int chan12 : 11;
	unsigned int chan13 : 11;
	unsigned int chan14 : 11;
	unsigned int chan15 : 11;
	u8 flags;
} __attribute__((__packed__)) sbusChannels_t;

#define SBUS_CHANNEL_DATA_LENGTH sizeof(sbusChannels_t)

struct sbusFrame_s
{
	u8 syncByte;
	sbusChannels_t channels;
	/**
	 * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
	 *
	 * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
	 * and
	 * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
	 */
	uint8_t endByte;
} __attribute__((__packed__));

typedef union sbusFrame_u
{
	u8 bytes[SBUS_FRAME_SIZE];
	struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s
{
	sbusFrame_t frame;
	timeUs_t startAtUs;
	uint8_t position;
	bool done;
} sbusFrameData_t;

void sbusInit(void);
u8 sbusFrameStatus(void);
void sbusReadRawRC(float *channelData, int count);
