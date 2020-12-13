#include "mpu9250.h"
#include "accgyro_mpu.h"
#include "system.h"

#include <stm32h7xx_hal.h>
#include <stdbool.h>

static void ak8963SlaveWriteRegister(spiInstance_t* spiInstance, uint8_t reg, uint8_t data);
static void ak8963SlaveReadRegisterBuffer(spiInstance_t* spiInstance, uint8_t reg, uint8_t *buf, uint8_t len);
static bool ak8963SlaveReadData(spiInstance_t* spiInstance, uint8_t *buf);

void mpu9250Init(imuDev_t* imu)
{
	// 16.4 LSB/(deg/s)
	imu->gyroScale = 16.4;
	imu->acc_1G = 2048;
	imu->accelScale = imu->acc_1G / 9.80665f;

	spiInstance_t *spi = &imu->spiInstance;

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_256);

	spiBusWriteRegister(spi, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
	HAL_Delay(50);

	uint8_t attemptsRemaining = 20;
	do {
		HAL_Delay(150);
		const uint8_t in = spiBusReadRegister(spi, MPU_RA_WHO_AM_I);
		if (in == MPU9250_WHO_AM_I_CONST) {
			break;
		}
		if (!attemptsRemaining) {
			return;
		}
	} while (attemptsRemaining--);

	spiBusWriteRegisterVerify(spi, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
	spiBusWriteRegisterVerify(spi, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
	spiBusWriteRegisterVerify(spi, MPU_RA_CONFIG, 0);
	spiBusWriteRegisterVerify(spi, MPU_RA_SMPLRT_DIV, 0);
	spiBusWriteRegisterVerify(spi, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
	spiBusWriteRegisterVerify(spi, MPU_RA_INT_PIN_CFG, MPU9250_BIT_BYPASS_EN | MPU9250_BIT_INT_ANYRD_2CLEAR);

	// MAG init
	{
		uint8_t asa[3];
		uint8_t status;

		spiBusWriteRegisterVerify(spi, MPU_RA_I2C_MST_CTRL, 0x0D);
		spiBusWriteRegisterVerify(spi, MPU_RA_USER_CTRL, 0x30);

		ak8963SlaveWriteRegister(spi, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);
		ak8963SlaveWriteRegister(spi, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM);
		ak8963SlaveReadRegisterBuffer(spi, AK8963_MAG_REG_ASAX, asa, sizeof(asa));

		imu->magGain.x = asa[0] + 128;
		imu->magGain.y = asa[1] + 128;
		imu->magGain.z = asa[2] + 128;

		ak8963SlaveWriteRegister(spi, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);

		// Clear status registers
		ak8963SlaveReadRegisterBuffer(spi, AK8963_MAG_REG_ST1, &status, 1);
		ak8963SlaveReadRegisterBuffer(spi, AK8963_MAG_REG_ST2, &status, 1);

		// Trigger first measurement
		ak8963SlaveWriteRegister(spi, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);
	}

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_2);
}

void mpu9250GetAccelSample(imuDev_t* imu, vector3_t *data)
{
	uint8_t buff[6];

	spiBusReadRegisterBuffer(&imu->spiInstance, MPU_RA_ACCEL_XOUT_H, buff, 6);

	data->x = (int16_t)((buff[0] << 8) | buff[1]);
	data->y = (int16_t)((buff[2] << 8) | buff[3]);
	data->z = (int16_t)((buff[4] << 8) | buff[5]);
}

void mpu9250GetGyroSample(imuDev_t* imu, vector3_t *data)
{
	static const uint8_t tx_buff[7] = { MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t buff[7];

	spiBusReadBuffer(&imu->spiInstance, tx_buff, buff, 7);

	data->x = (int16_t)((buff[1] << 8) | buff[2]);
	data->y = (int16_t)((buff[3] << 8) | buff[4]);
	data->z = (int16_t)((buff[5] << 8) | buff[6]);
}

static void ak8963SlaveWriteRegister(spiInstance_t* spiInstance, uint8_t reg, uint8_t data)
{
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_ADDR, AK8963_MAG_I2C_ADDRESS);
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_REG, reg);
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_DO, data);
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_CTRL, (1 & 0x0F) | I2C_SLV0_EN);
}

static void ak8963SlaveReadRegisterBuffer(spiInstance_t* spiInstance, uint8_t reg, uint8_t *buf, uint8_t len)
{
	// set I2C slave address for read
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_ADDR, AK8963_MAG_I2C_ADDRESS | READ_FLAG);
	// set I2C slave register
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_REG, reg);
	// read number of bytes
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);
	HAL_Delay(4);
	__disable_irq();
	// read I2C
	spiBusReadRegisterBuffer(spiInstance, MPU_RA_EXT_SENS_DATA_00, buf, len);
	__enable_irq();
}

bool ak8963Read(imuDev_t* imu, vector3_t *magData)
{
	uint8_t buff[7];

	bool ack = ak8963SlaveReadData(&imu->spiInstance, buff);

	uint8_t status2 = buff[6];
	if (!ack) {
		return false;
	}

	ak8963SlaveWriteRegister(&imu->spiInstance, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);

	if (status2 & ST2_MAG_SENSOR_OVERFLOW)
	{
		return false;
	}

	magData->x = constrain((int16_t)(buff[1] << 8 | buff[0]) * imu->magGain.x / 256, INT16_MIN, INT16_MAX);
	magData->y = constrain((int16_t)(buff[3] << 8 | buff[2]) * imu->magGain.y / 256, INT16_MIN, INT16_MAX);
	magData->z = constrain((int16_t)(buff[5] << 8 | buff[4]) * imu->magGain.z / 256, INT16_MIN, INT16_MAX);

	return true;
}

typedef struct queuedReadState_s {
	bool waiting;
	uint8_t len;
	uint32_t readStartedAt;
} queuedReadState_t;

static queuedReadState_t queuedRead = { false, 0, 0 };

static bool ak8963SlaveStartRead(spiInstance_t* spiInstance, uint8_t reg, uint8_t len)
{
	if (queuedRead.waiting) {
		return false;
	}

	queuedRead.len = len;

	// set I2C slave address for read
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_ADDR, AK8963_MAG_I2C_ADDRESS | READ_FLAG);
	// set I2C slave register
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_REG, reg);
	// read number of bytes
	spiBusWriteRegisterVerify(spiInstance, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);

	queuedRead.readStartedAt = micros();
	queuedRead.waiting = true;

	return true;
}

static uint32_t ak8963SlaveQueuedReadTimeRemaining(void)
{
	if (!queuedRead.waiting)
	{
		return 0;
	}

	int32_t timeSinceStarted = micros() - queuedRead.readStartedAt;
	int32_t timeRemaining = 8000 - timeSinceStarted;

	if (timeRemaining < 0)
	{
		return 0;
	}

	return timeRemaining;
}

static bool ak8963SlaveCompleteRead(spiInstance_t* spiInstance, uint8_t *buf)
{
	uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();

	if (timeRemaining > 0)
	{
		delayMicroseconds(timeRemaining);
	}

	queuedRead.waiting = false;

	spiBusReadRegisterBuffer(spiInstance, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);
	return true;
}

static bool ak8963SlaveReadData(spiInstance_t* spiInstance, uint8_t *buf)
{
	typedef enum {
		CHECK_STATUS = 0,
		WAITING_FOR_STATUS,
		WAITING_FOR_DATA
	} ak8963ReadState_e;

	static ak8963ReadState_e state = CHECK_STATUS;

	bool ack = false;

	// we currently need a different approach for the MPU9250 connected via SPI.
	// we cannot use the ak8963SlaveReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.

	bool retry = true;

restart:
	switch (state) {
	case CHECK_STATUS: {
		ak8963SlaveStartRead(spiInstance, AK8963_MAG_REG_ST1, 1);
		state = WAITING_FOR_STATUS;
		return false;
	}

	case WAITING_FOR_STATUS: {
		uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
		if (timeRemaining)
		{
			return false;
		}

		ack = ak8963SlaveCompleteRead(spiInstance, &buf[0]);

		uint8_t status = buf[0];

		if (!ack || (status & ST1_DATA_READY) == 0)
		{
			// too early. queue the status read again
			state = CHECK_STATUS;
			if (retry)
			{
				retry = false;
				goto restart;
			}
			return false;
		}

		// read the 6 bytes of data and the status2 register
		ak8963SlaveStartRead(spiInstance, AK8963_MAG_REG_HXL, 7);

		state = WAITING_FOR_DATA;
		return false;
	}

	case WAITING_FOR_DATA: {
		uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
		if (timeRemaining)
		{
			return false;
		}

		ack = ak8963SlaveCompleteRead(spiInstance, &buf[0]);
		state = CHECK_STATUS;
	}
	}

	return ack;
}
