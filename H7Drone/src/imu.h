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
	uint32_t cyclesRemaining;
} accelCalibration_t;

typedef struct
{
	vector3f_t sum;
	stdev_t var[3];
	uint32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct
{
	spiInstance_t spiInstance;
	imuInitFuncPtr initFn;
	bool dataReady;

	// Accel
	vector3_t accelTrims;
	float accelScale;
	uint16_t acc_1G;
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
	// degrees
	vector3f_t gyroData;

	bool areGyrosCalibrated;
	bool areAccelsCalibrated;
} imuData_t;

void imuInit(spiInstance_t spi, uint8_t index);
void imuUpdateGyroReadings(void);
void imuUpdateAccelReadings(void);
void imuUpdateMagReadings(void);
bool isGyroCalibrated(imuDev_t *imu);
bool isAccelCalibrated(imuDev_t *imu);
bool isAccelDataGood();
imuData_t* imuGetData(void);
void imuFilterGyro(timeUs_t currentTimeUs);
