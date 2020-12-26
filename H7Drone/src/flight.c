#include "flight.h"
#include "imu.h"

#include "maths.h"
#include <math.h>

timeUs_t prevTimeUs;

quaternion_t qt = QUATERNION_INITIALIZE;
quaternionProducts_t qP = QUATERNION_PRODUCTS_INITIALIZE;

float rMat[3][3];

int16_t roll, pitch, yaw;

float invSqrt(float x);

static void imuQuaternionComputeProducts(quaternion_t *quat, quaternionProducts_t *quatProd)
{
	quatProd->ww = quat->w * quat->w;
	quatProd->wx = quat->w * quat->x;
	quatProd->wy = quat->w * quat->y;
	quatProd->wz = quat->w * quat->z;
	quatProd->xx = quat->x * quat->x;
	quatProd->xy = quat->x * quat->y;
	quatProd->xz = quat->x * quat->z;
	quatProd->yy = quat->y * quat->y;
	quatProd->yz = quat->y * quat->z;
	quatProd->zz = quat->z * quat->z;
}

static bool isAccelGood(const imuDev_t *imu, float accMagnitudeSq)
{
	return true;
}

void flightUpdate(const timeUs_t currentTimeUs)
{
	const timeDelta_t delta = currentTimeUs - prevTimeUs;
	prevTimeUs = currentTimeUs;

	if (!imuData.areGyrosCalibrated) return;

	float dt = delta * 1e-6f;

	float gx = DEGREES_TO_RADIANS(imuData.gyroData.x);
	float gy = DEGREES_TO_RADIANS(imuData.gyroData.y);
	float gz = DEGREES_TO_RADIANS(imuData.gyroData.z);

	float ax = imuData.accelData.x;
	float ay = -imuData.accelData.y;
	float az = imuData.accelData.z;

	float ex = 0, ey = 0, ez = 0;

	float accMagnitudeSq = sq(ax) + sq(ay) + sq(az);
	if (isAccelDataGood() && accMagnitudeSq > 0.01f)
	{
		// Normalise accelerometer measurement
		accMagnitudeSq = invSqrt(accMagnitudeSq);
		ax *= accMagnitudeSq;
		ay *= accMagnitudeSq;
		az *= accMagnitudeSq;

		// Error is sum of cross product between estimated direction and measured direction of gravity
		ex += (ay * rMat[2][2] - az * rMat[2][1]);
		ey += (az * rMat[2][0] - ax * rMat[2][2]);
		ez += (ax * rMat[2][1] - ay * rMat[2][0]);
	}

	gx += 2.5f * ex;
	gy += 2.5f * ey;
	gz += 2.5f * ez;

	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);

	quaternion_t buffer;
	buffer.w = qt.w;
	buffer.x = qt.x;
	buffer.y = qt.y;
	buffer.z = qt.z;

	qt.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
	qt.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
	qt.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
	qt.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

	float recipNorm = invSqrt(sq(qt.w) + sq(qt.x) + sq(qt.y) + sq(qt.z));
	qt.w *= recipNorm;
	qt.x *= recipNorm;
	qt.y *= recipNorm;
	qt.z *= recipNorm;

	imuQuaternionComputeProducts(&qt, &qP);

	rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
	rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
	rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

	rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
	rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
	rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

	rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
	rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
	rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;

	roll = lrintf(atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
	pitch = lrintf(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (1800.0f / M_PIf));
	yaw = lrintf((-atan2f(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));
}
