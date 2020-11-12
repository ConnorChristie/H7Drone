#include "mpu6000.h"
#include "accgyro_mpu.h"
#include "system.h"

#include <stm32h7xx_hal.h>
#include <stdbool.h>

void mpu6000Init(imuDev_t* imu)
{
	// 16.4 LSB/(deg/s)
	imu->gyroScale = 2000.0f / (1 << 15);
	imu->acc_1G = 2048;
	imu->accelScale = imu->acc_1G / -ACCEL_1G;

	spiInstance_t *spi = &imu->spiInstance;

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_256);

	spiBusWriteRegister(spi, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	HAL_Delay(100);

	spiBusWriteRegister(spi, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	HAL_Delay(100);

	const uint8_t in = spiBusReadRegister(spi, MPU_RA_WHO_AM_I);
	uint8_t attemptsRemaining = 20;
	while (attemptsRemaining--)
	{
		if (in == MPU6000_WHO_AM_I_CONST)
		{
			break;
		}
	}

	spiBusWriteRegister(spi, MPU_RA_CONFIG, 0x00);
	delayMicroseconds(15);
	// Clock Source PPL with Z axis gyro reference
	spiBusWriteRegister(spi, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_PWR_MGMT_2, 0x00);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_SMPLRT_DIV, 0x00);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);      // INT_ANYRD_2CLEAR
	delayMicroseconds(15);
	spiBusWriteRegister(spi, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
	delayMicroseconds(15);

	spiSetSpeed(spi, SPI_BAUDRATEPRESCALER_2);
}

void mpu6000GetAccelSample(imuDev_t* imu, vector3_t *data)
{
	uint8_t buff[6];

	spiBusReadRegisterBuffer(&imu->spiInstance, MPU_RA_ACCEL_XOUT_H, buff, 6);

	data->x = (int16_t)((buff[0] << 8) | buff[1]);
	data->y = (int16_t)((buff[2] << 8) | buff[3]);
	data->z = (int16_t)((buff[4] << 8) | buff[5]);
}

void mpu6000GetGyroSample(imuDev_t* imu, vector3_t *data)
{
	static const uint8_t dataToSend[7] = { MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t buff[7];

	spiBusReadBuffer(&imu->spiInstance, dataToSend, buff, 7);

	data->x = (int16_t)((buff[1] << 8) | buff[2]);
	data->y = (int16_t)((buff[3] << 8) | buff[4]);
	data->z = (int16_t)((buff[5] << 8) | buff[6]);
}
