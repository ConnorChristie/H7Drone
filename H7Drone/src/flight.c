#include "flight.h"
#include "imu.h"

#include "maths.h"
#include <math.h>

timeUs_t prevTimeUs;

float p, q, r;
bool isInitialized;

vector3f_t integrals;

quaternion_t qt = QUATERNION_INITIALIZE;
quaternionProducts_t qP = QUATERNION_PRODUCTS_INITIALIZE;

float rMat[3][3];

int16_t roll, pitch, yaw;

#define sampleFreq	8000.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

volatile float beta = betaDef; 								// 2 * proportional gain (Kp)
volatile float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f; 	// quaternion of sensor frame relative to auxiliary frame

float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

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
	
}

void flightUpdate(const timeUs_t currentTimeUs)
{
	const timeDelta_t delta = currentTimeUs - prevTimeUs;
	prevTimeUs = currentTimeUs;

	imuData_t *imuData = imuGetData();
	if (!imuData->areGyrosCalibrated) return;

	float dt = delta * 1e-6f;

	float gx = DEGREES_TO_RADIANS(-imuData->gyroData.x);
	float gy = DEGREES_TO_RADIANS(imuData->gyroData.y);
	float gz = DEGREES_TO_RADIANS(imuData->gyroData.z);

	float ax = imuData->accelData.x;
	float ay = -imuData->accelData.y;
	float az = imuData->accelData.z;

	float ex = 0, ey = 0, ez = 0;

	float accMagnitudeSq = sq(ax) + sq(ay) + sq(az);
	if (isAccelDataGood() && accMagnitudeSq > 0.01f) {
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

//	roll += imuData->gyroData.x * timeScale;
//	pitch += imuData->gyroData.y * timeScale;

//	roll += pitch * sinf(imuData->gyroData.z * timeScale);
//	pitch += roll * sinf(imuData->gyroData.z * timeScale);

//	MadgwickAHRSupdateIMU(
//		imuData->gyroData.x,
//		imuData->gyroData.y,
//		imuData->gyroData.z,
//		imuData->accelData.y, imuData->accelData.x, imuData->accelData.z);

//	const timeDelta_t delta = currentTimeUs - prevTimeUs;
//	prevTimeUs = currentTimeUs;
//
//	float timeScale = delta * 1e-6f;
//
//	float phi_hat_acc = atan2f(imuData->accelData.y, sqrtf(imuData->accelData.x * imuData->accelData.x + imuData->accelData.z * imuData->accelData.z));
//	float theta_hat_acc = atan2f(-imuData->accelData.x, sqrtf(imuData->accelData.y * imuData->accelData.y + imuData->accelData.z * imuData->accelData.z));
//
//	//	if (!isInitialized)
//	//	{
//	//		roll = angle_roll_acc;
//	//		pitch = angle_pitch_acc;
//	//		isInitialized = true;
//	//	}
//
//	// Accumulate the traveled angles.
//	p += imuData->gyroData.x * timeScale;
//	q += imuData->gyroData.y * timeScale;
//	r += imuData->gyroData.z * timeScale;
//
//	// Calculate Euler angle derivatives 
//	float phi_dot = p + sinf(phi_hat) * tanf(theta_hat) * q + cosf(phi_hat) * tanf(theta_hat) * r;
//	float theta_dot = cosf(phi_hat) * q - sinf(phi_hat) * r;
//
//	// Update complimentary filter
//	phi_hat = (1 - alpha) * (phi_hat + timeScale * phi_dot) + alpha * phi_hat_acc;
//	theta_hat = (1 - alpha) * (theta_hat + timeScale * theta_dot) + alpha * theta_hat_acc;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz);
	qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy);
	qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx);
	qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * qw;
		_2q1 = 2.0f * qx;
		_2q2 = 2.0f * qy;
		_2q3 = 2.0f * qz;
		_4q0 = 4.0f * qw;
		_4q1 = 4.0f * qx;
		_4q2 = 4.0f * qy;
		_8q1 = 8.0f * qx;
		_8q2 = 8.0f * qy;
		q0q0 = qw * qw;
		q1q1 = qx * qx;
		q2q2 = qy * qy;
		q3q3 = qz * qz;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * qx - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * qy + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * qz - _2q1 * ax + 4.0f * q2q2 * qz - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	qw += qDot1 * (1.0f / sampleFreq);
	qx += qDot2 * (1.0f / sampleFreq);
	qy += qDot3 * (1.0f / sampleFreq);
	qz += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
	qw *= recipNorm;
	qx *= recipNorm;
	qy *= recipNorm;
	qz *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
