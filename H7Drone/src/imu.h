#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "maths.h"
#include "scheduler.h"

struct imuDev_t;

typedef void(*imuInitFuncPtr)(struct imuDev_t *imu);
typedef void(*imuReadAccelFuncPtr)(struct imuDev_t *imu, vector3_t *data);
typedef void(*imuReadGyroFuncPtr)(struct imuDev_t *imu, vector3_t *data);

typedef struct
{
	vector3l_t sum;
	u32 cyclesRemaining;
} accelCalibration_t;

typedef struct
{
	vector3f_t sum;
	stdev_t var[3];
	u32 cyclesRemaining;
} gyroCalibration_t;

typedef struct
{
	spiInstance_t spiInstance;
	imuInitFuncPtr initFn;
	bool dataReady;

	// Accel
	vector3_t accelTrims;
	float accelScale;
	u16 acc_1G;
	float acc_1G_rec;
	accelCalibration_t accelCalibration;
	imuReadAccelFuncPtr accelReadFn;

	// Gyro
	vector3f_t gyroZero;
	float gyroScale;
	gyroCalibration_t gyroCalibration;
	imuReadGyroFuncPtr gyroReadFn;

	// Mag
	vector3_t magGain;
	vector3_t magDataRaw;
} imuDev_t;

typedef struct
{
	// m/s^2
	vector3f_t accelData;
	// degrees/s
	vector3f_t gyroData;

	bool areGyrosCalibrated;
	bool areAccelsCalibrated;
} imuData_t;

#define FIR_FILTER_LENGTH 64

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = { 0.0004167f, 0.0004284f, 0.0004611f, 0.0005148f, 0.0005891f, 0.0006834f, 0.0007969f, 0.0009286f, 0.0010774f, 0.0012419f, 0.0014206f, 0.0016118f, 0.0018137f, 0.0020244f, 0.0022419f, 0.0024641f, 0.0026889f, 0.0029140f, 0.0031373f, 0.0033566f, 0.0035697f, 0.0037746f, 0.0039692f, 0.0041516f, 0.0043200f, 0.0044728f, 0.0046085f, 0.0047256f, 0.0048231f, 0.0048999f, 0.0049553f, 0.0049888f, 0.0050000f, 0.0049888f, 0.0049553f, 0.0048999f, 0.0048231f, 0.0047256f, 0.0046085f, 0.0044728f, 0.0043200f, 0.0041516f, 0.0039692f, 0.0037746f, 0.0035697f, 0.0033566f, 0.0031373f, 0.0029140f, 0.0026889f, 0.0024641f, 0.0022419f, 0.0020244f, 0.0018137f, 0.0016118f, 0.0014206f, 0.0012419f, 0.0010774f, 0.0009286f, 0.0007969f, 0.0006834f, 0.0005891f, 0.0005148f, 0.0004611f, 0.0004284f };

typedef struct
{
	float buffer[FIR_FILTER_LENGTH];
	u8 bufferIndex;
} firFilter_s;

extern imuData_t imuData;

void imuInit(spiInstance_t spi, u8 index);
void imuUpdateGyroReadings(void);
void imuUpdateAccelReadings(void);
void imuUpdateMagReadings(void);
bool isGyroCalibrated(imuDev_t *imu);
bool isAccelCalibrated(imuDev_t *imu);
bool isAccelDataGood();
imuData_t* imuGetData(void);
void imuFilterGyro(timeUs_t currentTimeUs);
