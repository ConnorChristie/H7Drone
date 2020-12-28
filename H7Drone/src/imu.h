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

typedef enum
{
    ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11
} sensorAlign_e;

typedef struct
{
	sensorAlign_e alignment;
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
	vector3_t rawAccelData;
	// degrees/s
	vector3f_t gyroData;
	vector3_t rawGyroData;

	bool areGyrosCalibrated;
	bool areAccelsCalibrated;
} imuData_t;

extern imuData_t imuData;

void imuInit(spiInstance_t spi, u8 index, sensorAlign_e alignment);
void imuUpdateGyroReadings(void);
void imuUpdateAccelReadings(void);
void imuUpdateMagReadings(void);
bool isGyroCalibrated(imuDev_t *imu);
bool isAccelCalibrated(imuDev_t *imu);
bool isAccelDataGood();
imuData_t* imuGetData(void);
void imuFilterGyro(timeUs_t currentTimeUs);
