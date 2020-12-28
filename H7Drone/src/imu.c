#include "imu.h"
#include "platform.h"
#include "system.h"
#include "flight.h"

#include "drivers/mpu6000.h"
#include "drivers/icm20602.h"

FAST_DATA_ZERO_INIT imuData_t imuData;
u8 imuDeviceCount = 0;
static imuDev_t *imuDevices = NULL;

static void performAccelCalibration(imuDev_t *device, const vector3_t rawAccelData);
static void performGyroCalibration(imuDev_t *device, const vector3_t rawGyroData, u8 gyroMovementgyroCalibrationThreshold);
static u32 accelCalculateCalibratingCycles(void);
static u32 gyroCalculateCalibratingCycles(void);
void alignSensorViaRotation(vector3f_t *dest, sensorAlign_e rotation);

void onDataReady(void *ctx)
{
	imuDev_t *imu = (imuDev_t*)ctx;

	imu->dataReady = true;
}

void imuInit(spiInstance_t spi, u8 index, sensorAlign_e alignment)
{
	imuDeviceCount++;

	if (imuDevices == NULL)
	{
		imuDevices = malloc(imuDeviceCount * sizeof(imuDev_t));
	}
	else
	{
		imuDevices = realloc(imuDevices, imuDeviceCount * sizeof(imuDev_t));
	}

	imuDev_t *imu = &imuDevices[index];
	memset(imu, 0, sizeof(imuDev_t));

	imu->alignment = alignment;
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
	// Overridden down below
	imuData.areGyrosCalibrated = true;

	for (u8 i = 0; i < imuDeviceCount; i++)
	{
		imuDev_t *imu = &imuDevices[i];

		if (!imu->dataReady)
		{
			continue;
		}

		imu->dataReady = false;
		imu->gyroReadFn((struct imuDev_t*)imu, &imuData.rawGyroData);

		bool caliibrated = isGyroCalibrated(imu);
		imuData.areGyrosCalibrated &= caliibrated;

		if (caliibrated)
		{
			imuData.gyroData.x = (imuData.rawGyroData.x - imu->gyroZero.x) * imu->gyroScale;
			imuData.gyroData.y = (imuData.rawGyroData.y - imu->gyroZero.y) * imu->gyroScale;
			imuData.gyroData.z = (imuData.rawGyroData.z - imu->gyroZero.z) * imu->gyroScale;

			alignSensorViaRotation(&imuData.gyroData, imu->alignment);
		}
		else
		{
			performGyroCalibration(imu, imuData.rawGyroData, 48);
		}
	}
}

void imuFilterGyro(timeUs_t currentTimeUs)
{
}

void imuUpdateAccelReadings(void)
{
	// Overridden down below
	imuData.areAccelsCalibrated = true;

	for (u8 i = 0; i < imuDeviceCount; i++)
	{
		imuDev_t *imu = &imuDevices[i];
		imu->accelReadFn((struct imuDev_t*)imu, &imuData.rawAccelData);

		bool caliibrated = isAccelCalibrated(imu);
		imuData.areAccelsCalibrated &= caliibrated;

		if (caliibrated)
		{
			imuData.accelData.x = (float)(imuData.rawAccelData.x - imu->accelTrims.x) / imu->accelScale;
			imuData.accelData.y = (float)(imuData.rawAccelData.y - imu->accelTrims.y) / imu->accelScale;
			imuData.accelData.z = (float)(imuData.rawAccelData.z - imu->accelTrims.z) / imu->accelScale;
		}
		else
		{
			performAccelCalibration(imu, imuData.rawAccelData);
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

static u32 gyroCalculateCalibratingCycles(void)
{
	return 1000;
}

static u32 accelCalculateCalibratingCycles(void)
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

static void performGyroCalibration(imuDev_t *device, const vector3_t rawGyroData, u8 gyroMovementgyroCalibrationThreshold)
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

FAST_CODE void alignSensorViaRotation(vector3f_t *dest, sensorAlign_e rotation)
{
	const float x = dest->x;
	const float y = dest->y;
	const float z = dest->z;

	switch (rotation) {
	default:
	case CW0_DEG:
		dest->x = x;
		dest->y = y;
		dest->z = z;
		break;
	case CW90_DEG:
		dest->x = y;
		dest->y = -x;
		dest->z = z;
		break;
	case CW180_DEG:
		dest->x = -x;
		dest->y = -y;
		dest->z = z;
		break;
	case CW270_DEG:
		dest->x = -y;
		dest->y = x;
		dest->z = z;
		break;
	case CW0_DEG_FLIP:
		dest->x = -x;
		dest->y = y;
		dest->z = -z;
		break;
	case CW90_DEG_FLIP:
		dest->x = y;
		dest->y = x;
		dest->z = -z;
		break;
	case CW180_DEG_FLIP:
		dest->x = x;
		dest->y = -y;
		dest->z = -z;
		break;
	case CW270_DEG_FLIP:
		dest->x = -y;
		dest->y = -x;
		dest->z = -z;
		break;
	}
}
