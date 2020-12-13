#include "imu.h"
#include "system.h"
#include "flight.h"
#include "drivers/mpu6000.h"
#include "drivers/icm20602.h"

#include <string.h>
#include <stdlib.h>

imuData_t imuData;
uint8_t imuDeviceCount = 0;
imuDev_t *imuDevices = NULL;

static void performAccelCalibration(imuDev_t *device, const vector3_t rawAccelData);
static void performGyroCalibration(imuDev_t *device, const vector3_t rawGyroData, uint8_t gyroMovementgyroCalibrationThreshold);
static uint32_t accelCalculateCalibratingCycles(void);
static uint32_t gyroCalculateCalibratingCycles(void);

void onDataReady(void *ctx)
{
	imuDev_t *imu = (imuDev_t*)ctx;

	imu->dataReady = true;
}

void imuInit(spiInstance_t spi, uint8_t index)
{
	imuDeviceCount++;

	if (imuDevices == NULL)
		imuDevices = malloc(imuDeviceCount * sizeof(imuDev_t));
	else
		imuDevices = realloc(imuDevices, imuDeviceCount * sizeof(imuDev_t));

	imuDev_t *imu = &imuDevices[index];
	memset(imu, 0, sizeof(imuDev_t));

	imu->spiInstance = spi;
	imu->accelCalibration.cyclesRemaining = accelCalculateCalibratingCycles();
	imu->gyroCalibration.cyclesRemaining = gyroCalculateCalibratingCycles();

	if (index == 0)
	{
		imu->initFn = (imuInitFuncPtr)mpu6000Init;
		imu->accelReadFn = (imuReadAccelFuncPtr)mpu6000GetAccelSample;
		imu->gyroReadFn = (imuReadGyroFuncPtr)mpu6000GetGyroSample;

		enableInterrupt(2, (interruptFuncPtr)onDataReady, (void*)imu);
	}
	else
	{
		imu->initFn = (imuInitFuncPtr)icm20602Init;
		imu->accelReadFn = (imuReadAccelFuncPtr)icm20602GetAccelSample;
		imu->gyroReadFn = (imuReadGyroFuncPtr)icm20602GetGyroSample;

		//enableInterrupt(6, (interruptFuncPtr)onDataReady, (void*)imu);
	}

	imu->initFn((struct imuDev_t*)imu);

	imu->acc_1G_rec = 1.0f / imu->acc_1G;
}

void imuUpdateGyroReadings(void)
{
	vector3_t rawGyroData;
	imuData.areGyrosCalibrated = true;

	for (uint8_t i = 0; i < imuDeviceCount; i++)
	{
		imuDev_t *imu = &imuDevices[i];

		if (!imu->dataReady)
		{
			continue;
		}

		imu->dataReady = false;
		imu->gyroReadFn((struct imuDev_t*)imu, &rawGyroData);

		bool caliibrated = isGyroCalibrated(imu);
		imuData.areGyrosCalibrated &= caliibrated;

		if (caliibrated)
		{
			imuData.gyroData.x = (rawGyroData.x - imu->gyroZero.x) * imu->gyroScale;
			imuData.gyroData.y = (rawGyroData.y - imu->gyroZero.y) * imu->gyroScale;
			imuData.gyroData.z = (rawGyroData.z - imu->gyroZero.z) * imu->gyroScale;
		}
		else
		{
			performGyroCalibration(imu, rawGyroData, 48);
		}
	}
	
//	imuData.sampleSum.x += imuData.gyroData.x;
//	imuData.sampleSum.y += imuData.gyroData.y;
//	imuData.sampleSum.z += imuData.gyroData.z;
//	imuData.sampleCount++;
}

vector3f_t accumulatedMeasurements;
vector3f_t gyroPrevious;
int accumulatedMeasurementCount;

void imuFilterGyro(timeUs_t currentTimeUs)
{
	accumulatedMeasurements.x += 0.5f * (gyroPrevious.x + imuData.gyroData.x) * 125;
	accumulatedMeasurements.y += 0.5f * (gyroPrevious.y + imuData.gyroData.y) * 125;
	accumulatedMeasurements.z += 0.5f * (gyroPrevious.z + imuData.gyroData.z) * 125;

	memcpy(&gyroPrevious.xyz, &imuData.gyroData.xyz, sizeof(imuData.gyroData.xyz));
	accumulatedMeasurementCount++;
}

// TODO: Finish getting accumated average

void imuUpdateAccelReadings(void)
{
	vector3_t rawAccelData;
	imuData.areAccelsCalibrated = true;

	for (uint8_t i = 0; i < imuDeviceCount; i++)
	{
		imuDev_t *imu = &imuDevices[i];
		imu->accelReadFn((struct imuDev_t*)imu, &rawAccelData);

		bool caliibrated = isAccelCalibrated(imu);
		imuData.areAccelsCalibrated &= caliibrated;

		if (caliibrated)
		{
			rawAccelData.x -= imu->accelTrims.x;
			rawAccelData.y -= imu->accelTrims.y;
			rawAccelData.z -= imu->accelTrims.z;

			imuData.accelData.x = (float)rawAccelData.x / imu->accelScale;
			imuData.accelData.y = (float)rawAccelData.y / imu->accelScale;
			imuData.accelData.z = (float)rawAccelData.z / imu->accelScale;
			
			int a = 1;
		}
		else
		{
			performAccelCalibration(imu, rawAccelData);
		}
	}
}

void imuUpdateMagReadings(void)
{
	//ak8963Read(&imuDevices[0], &imuDevices[0].magDataRaw);
}

bool isGyroCalibrated(imuDev_t *imu)
{
	return imu->gyroCalibration.cyclesRemaining == 0;
}

bool isAccelCalibrated(imuDev_t *imu)
{
	return imu->accelCalibration.cyclesRemaining == 0;
}

bool isAccelDataGood()
{
	float accMagnitudeSq = sq(imuData.accelData.x) + sq(imuData.accelData.y) + sq(imuData.accelData.z);

	accMagnitudeSq *= sq(1.0f / ACCEL_1G);

	return accMagnitudeSq > 0.81f && 1.21f > accMagnitudeSq;
}

imuData_t* imuGetData(void)
{
	return &imuData;
}

static uint32_t gyroCalculateCalibratingCycles(void)
{
	return 1000;
}

static uint32_t accelCalculateCalibratingCycles(void)
{
	return 400;
}

static void performAccelCalibration(imuDev_t *device, const vector3_t rawAccelData)
{
	if (isAccelCalibrated(device)) return;

	for (int axis = 0; axis < 3; axis++)
	{
		// Reset g[axis] at start of gyroCalibration
		if (device->accelCalibration.cyclesRemaining == accelCalculateCalibratingCycles())
		{
			device->accelCalibration.sum.xyz[axis] = 0;
		}

		device->accelCalibration.sum.xyz[axis] += rawAccelData.xyz[axis];

		if (device->accelCalibration.cyclesRemaining == 1)
		{
			device->accelTrims.xyz[axis] = (device->accelCalibration.sum.xyz[axis] + (accelCalculateCalibratingCycles() / 2)) / accelCalculateCalibratingCycles();

			if (axis == 2)
			{
				device->accelTrims.xyz[axis] += device->acc_1G;
			}
		}
	}

	device->accelCalibration.cyclesRemaining--;
}

static void performGyroCalibration(imuDev_t *device, const vector3_t rawGyroData, uint8_t gyroMovementgyroCalibrationThreshold)
{
	if (isGyroCalibrated(device)) return;

	for (int axis = 0; axis < 3; axis++)
	{
		// Reset g[axis] at start of gyroCalibration
		if (device->gyroCalibration.cyclesRemaining == gyroCalculateCalibratingCycles())
		{
			device->gyroCalibration.sum.xyz[axis] = 0.0f;
			device->gyroCalibration.var[axis].m_n = 0;
		}

		// Sum up CALIBRATING_GYRO_TIME_US readings
		device->gyroCalibration.sum.xyz[axis] += rawGyroData.xyz[axis];
		devPush(&device->gyroCalibration.var[axis], rawGyroData.xyz[axis]);

		if (device->gyroCalibration.cyclesRemaining == 1)
		{
			const float stddev = devStandardDeviation(&device->gyroCalibration.var[axis]);

			// check deviation and startover in case the model was moved
			if (gyroMovementgyroCalibrationThreshold && stddev > gyroMovementgyroCalibrationThreshold)
			{
				device->gyroCalibration.cyclesRemaining = gyroCalculateCalibratingCycles();
				return;
			}

			// please take care with exotic boardalignment !!
			device->gyroZero.xyz[axis] = device->gyroCalibration.sum.xyz[axis] / gyroCalculateCalibratingCycles();
		}
	}

	device->gyroCalibration.cyclesRemaining--;
}

//static void filterGyro(imuData_t *data)
//{
//	for (int axis = 0; axis < 3; axis++)
//	{
//		// downsample the individual gyro samples
//		float gyroADCf = 0;
//
//		// using simple average for downsampling
//		if (data->sampleCount)
//		{
//			gyroADCf = data->sampleSum[axis] / data->sampleCount;
//		}
//		data->sampleSum[axis] = 0;
//
//		// apply static notch filters and software lowpass filters
//		gyroADCf = notchFilter1((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
//		gyroADCf = notchFilter2((filter_t *)&gyro.notchFilter2[axis], gyroADCf);
//		gyroADCf = lowpassFilter((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);
//
//		data->gyroDataFiltered.xyz[axis] = gyroADCf;
//	}
//	data->sampleCount = 0;
//}
